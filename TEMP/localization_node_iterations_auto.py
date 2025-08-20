import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import transformations as tf
import std_msgs.msg
import pandas as pd  
import subprocess
import os
import signal
import time
import re
from threading import Event, Thread

class LocalizationNode(Node):
    def __init__(self, icp_max_iterations, done_event):
        super().__init__('localization_node')
        self.done_event = done_event  
        print(f"Inicializando el nodo de localización con {icp_max_iterations} iteraciones máximas de ICP...")

        self.icp_max_iterations = icp_max_iterations
        self.save_path = f"/odom_data/odom_data_{icp_max_iterations}_iterations.csv"  # Ruta al fichero de datos de salida
        # MAPA 1
        self.rosbag_path = "/rosbags/rosbag1_fl"  # Ruta a la rosbag
        self.map_file = "/MAPAS/mapa_rosbag1_voxel0_01.pcd"  # Ruta al mapa
        
        self.map_cloud = o3d.io.read_point_cloud(self.map_file)
        if self.map_cloud.is_empty():
            self.get_logger().error(f"No se pudo cargar el archivo de mapa: {self.map_file}")
            return
        print("Archivo de mapa cargado correctamente.")
        self.voxel_size_map = 0.1 # Tamaño de voxelizado del mapa
        self.voxel_size_cloud = 0.1
        self.map_cloud = self.map_cloud.voxel_down_sample(self.voxel_size_map)
        print("Voxelización del mapa completada.")
        
        self.max_time_icp = 0
        self.rate = 0.7
        self.initial_transformation = np.identity(4)
        self.last_odom_msg = None
        self.erroneous_x = None
        self.erroneous_y = None
        self.enable_voxelization = True
        self.data = pd.DataFrame(columns=[
            'X', 'Y', '0. pre X', '0. pre Y',
            '0. post X', '0. post Y',
            'ICP Translation X', 'ICP Translation Y',
            'Error pre X', 'Error pre Y',
            'Error post X', 'Error post Y',
            'ECM pre', 'ECM post'
        ])
        
        # Parámetros de error para la odometría
        self.offset_x = 1.0
        self.offset_y = 1.0
        self.offset_z = 0.0
        self.offset_yaw = 0.0
        self.noise_mu = 0.0  # Media del ruido
        self.noise_sigma = 0  # Desviación estándar del ruido
        
        # Parámetros para no procesar todas las nubes de puntos (solo cada clouds_threshold)
        self.clouds_counter = 0
        self.clouds_threshold = 9

        self.bag_process = None
        self.bag_paused = False
        self.rosbag_duration = self.get_rosbag_duration(self.rosbag_path)
        self.start_time = None
        self.active_play_time = 0  # Acumulación de tiempo activo de reproducción
        self.last_resume_time = None  # Último tiempo al reanudar la reproducción

        qos_profile_reliable = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        qos_profile_best_effort = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        self.odom_subscription = self.create_subscription(Odometry, '/Odometry', self.odom_callback, qos_profile_reliable)
        self.subscription = self.create_subscription(PointCloud2, '/ouster/points', self.points_callback, qos_profile_best_effort)
        self.cloud_publisher = self.create_publisher(PointCloud2, '/cloudICP', qos_profile_best_effort)
        self.original_cloud_publisher = self.create_publisher(PointCloud2, '/cloud', qos_profile_best_effort)
        self.map_publisher = self.create_publisher(PointCloud2, '/map', qos_profile_reliable)
        self.odom_pre_icp_publisher = self.create_publisher(Odometry, '/odometryPreICP', qos_profile_reliable)
        self.odom_post_icp_publisher = self.create_publisher(Odometry, '/odometryPostICP', qos_profile_reliable)
        self.publish_map(self.map_cloud)

        # Reproducción de la rosbag
        if os.path.exists(self.rosbag_path):
            self.start_bag()
        else:
            self.get_logger().error(f"No existe la rosbag: {self.rosbag_path}")

    def get_rosbag_duration(self, rosbag_path):
        try:
            result = subprocess.run(["ros2", "bag", "info", rosbag_path], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if "Duration:" in line:
                    duration_str = line.split("Duration:")[1].strip().split()[0]
                    duration_str = re.sub('[^0-9:.]', '', duration_str)  
                    parts = list(map(float, duration_str.split(':')))
                    h, m, s = 0, 0, parts[0]
                    duration = h * 3600 + m * 60 + s
                    return duration
        except Exception as e:
            self.get_logger().error(f"Failed to get rosbag duration: {e}")
            return None

    def start_bag(self):
        self.bag_process = subprocess.Popen(["ros2", "bag", "play", "--rate", "0.1", self.rosbag_path])
        self.last_resume_time = time.time()
        print("Started rosbag playback.")

    def pause_bag(self):
        if self.bag_process and not self.bag_paused:
            self.bag_process.send_signal(signal.SIGSTOP)
            self.bag_paused = True
            if self.last_resume_time:
                self.active_play_time += time.time() - self.last_resume_time
            print("Paused rosbag playback.")

    def resume_bag(self):
        if self.bag_process and self.bag_paused:
            self.bag_process.send_signal(signal.SIGCONT)
            self.bag_paused = False
            self.last_resume_time = time.time()  # Reiniciar la cuenta de tiempo con la reproducción de la rosbag activa
            print("Resumed rosbag playback.")

    def publish_map(self, map_cloud):  # Publica el mapa completo
        points = np.asarray(map_cloud.points)
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'os_map'
        ros_cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.map_publisher.publish(ros_cloud)
    
    def generate_noise(self):
        return np.random.normal(self.noise_mu, self.noise_sigma)

    def odom_callback(self, msg):  # Guarda la odometría real y la errónea para cada mensaje de odometría
        self.last_odom_msg = msg
        trans = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        z_rotation = np.arctan2(2.0 * (rot.w * rot.z + rot.x * rot.y), 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))
        z_rotation = self.normalize_angle(z_rotation)
        # Aplicar offset y ruido
        noise_x = self.generate_noise()
        noise_y = self.generate_noise()
        #noise_z = self.generate_noise()
        odom_translation = np.array([trans.x + self.offset_x + noise_x, trans.y + self.offset_y + noise_y, trans.z + self.offset_z])
        odom_rotation_matrix = tf.rotation_matrix(z_rotation, (0, 0, 1))
        self.initial_transformation = np.dot(tf.translation_matrix(odom_translation), odom_rotation_matrix)
        self.initial_x, self.initial_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.erroneous_x = msg.pose.pose.position.x + self.offset_x + noise_x
        self.erroneous_y = msg.pose.pose.position.y + self.offset_y + noise_y
        # Publicar odometría con ruido
        self.publish_odometry(self.erroneous_x, self.erroneous_y, self.last_odom_msg.pose.pose.orientation, 'os_map', self.odom_pre_icp_publisher)

    def normalize_angle(self, angle):  # Normaliza el ángulo de giro a [-pi,pi]
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def points_callback(self, msg):  # Procesa los mensajes de nubes de puntos del LiDAR, se lanza ICP y se guardan los valores de interés
        if self.last_odom_msg is None:
            self.get_logger().error("No se han recibido mensajes de odometría aún. No se ejecuta la corrección por ICP.")
            return
        
        points = np.array([np.array((point[0], point[1], point[2])) for point in point_cloud2.read_points(msg, skip_nans=True)], dtype=np.float32)
        original_cloud = o3d.geometry.PointCloud()
        original_cloud.points = o3d.utility.Vector3dVector(points)
        if self.enable_voxelization:
            original_cloud = original_cloud.voxel_down_sample(self.voxel_size_cloud)
        original_cloud.transform(self.initial_transformation)
        self.publish_cloud(original_cloud, 'original')
        
        self.clouds_counter += 1
        if self.clouds_counter >= self.clouds_threshold:
            self.clouds_counter = 0
            self.pause_bag()
            
            icp_transformation = self.perform_icp(original_cloud, self.map_cloud)
            original_cloud.transform(icp_transformation)
            self.publish_cloud(original_cloud, 'transformed')
            
            icp_translation_x, icp_translation_y = icp_transformation[0, 3], icp_transformation[1, 3]
            icp_x = self.erroneous_x + icp_translation_x
            icp_y = self.erroneous_y + icp_translation_y
            error_erroneous_x = self.erroneous_x - self.initial_x
            error_erroneous_y = self.erroneous_y - self.initial_y
            error_icp_x = icp_x - self.initial_x
            error_icp_y = icp_y - self.initial_y
            ecm_pre = (error_erroneous_x ** 2 + error_erroneous_y ** 2) / 2
            ecm_post = (error_icp_x ** 2 + error_icp_y ** 2) / 2

            new_row = {
                'X': self.initial_x, 'Y': self.initial_y,
                '0. pre X': self.erroneous_x, '0. pre Y': self.erroneous_y,
                '0. post X': icp_x, '0. post Y': icp_y,
                'ICP Translation X': icp_translation_x, 'ICP Translation Y': icp_translation_y,
                'Error pre X': error_erroneous_x, 'Error pre Y': error_erroneous_y,
                'Error post X': error_icp_x, 'Error post Y': error_icp_y,
                'ECM pre': ecm_pre, 'ECM post': ecm_post
            }

            new_df = pd.DataFrame(new_row, index=[0])
            self.data = pd.concat([self.data, new_df], ignore_index=True)
            self.data.to_csv(self.save_path, index=False, sep=';')
            self.publish_odometry(icp_x, icp_y, self.last_odom_msg.pose.pose.orientation, 'os_map', self.odom_post_icp_publisher)
            self.resume_bag()

    def perform_icp(self, source_cloud, target_cloud):
        print(f"Calculando ICP con {self.icp_max_iterations} iteraciones máximas")
        threshold = 1.5
        trans_init = np.identity(4)
        self.get_logger().info("Comienza ICP")
        icp_start = time.time()
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_cloud, target_cloud, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.icp_max_iterations
            )
        )
        icp_end = time.time()
        icp_time = icp_end - icp_start
        self.max_time_icp = max(self.max_time_icp, icp_time)
        self.get_logger().info("Finaliza ICP")
        print(f"Tiempo de ejecución ICP: {icp_time:.4f} segundos")
        print(f"Máximo tiempo de ejecución ICP hasta el momento: {self.max_time_icp:.4f} segundos")
        transformation_matrix = reg_p2p.transformation
        translation = transformation_matrix[:3, 3]
        rotation = transformation_matrix[:3, :3]
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        yaw_degrees = yaw * 180 / np.pi
        print(f"Traslación ICP: {translation}, Yaw: {yaw_degrees} grados")
        return transformation_matrix

    def publish_cloud(self, cloud, cloud_type):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'os_map'
        points = np.asarray(cloud.points)
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
        if cloud_type == 'original':
            self.original_cloud_publisher.publish(cloud_msg)
        elif cloud_type == 'transformed':
            self.cloud_publisher.publish(cloud_msg)

    def publish_odometry(self, x, y, orientation, frame_id, publisher):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = frame_id
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = orientation
        publisher.publish(odom_msg)

    def get_bag_progress(self):
        if self.rosbag_duration > 0:
            if not self.bag_paused and self.last_resume_time:
                # Actualiza el tiempo activo si aún está reproduciendo
                current_active_time = time.time() - self.last_resume_time
                total_active_time = (self.active_play_time + current_active_time) * self.rate 
            else:
                total_active_time = self.active_play_time * self.rate
            
            progress_percentage = (total_active_time / self.rosbag_duration) * 100
            if progress_percentage >= 100:  # 100 % indica que la rosbag ha terminado
                print("Rosbag completada.")
                self.bag_process.send_signal(signal.SIGTERM)  # Terminar el proceso de rosbag
                self.done_event.set()  
                return True
            else:
                print(f"Porcentaje completado: {progress_percentage:.2f}%")
            return False
        return False

def main(args=None):
    icp_iterations_list = [1,2,20]  # Lista de iteraciones máximas de ICP a probar

    for icp_max_iterations in icp_iterations_list:
        rclpy.init(args=args)
        done_event = Event()
        node = LocalizationNode(icp_max_iterations, done_event)
        
        def check_progress():
            while rclpy.ok():
                if node.get_bag_progress():
                    return  # Terminar el thread cuando la rosbag se haya completado
                rclpy.spin_once(node, timeout_sec=5)
        
        progress_thread = Thread(target=check_progress)
        progress_thread.start()

        done_event.wait()  # Esperar a que la rosbag se complete para esta iteración

        node.destroy_node()
        rclpy.shutdown()
        progress_thread.join()

if __name__ == '__main__':
    main()
