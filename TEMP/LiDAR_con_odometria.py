import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import transformations as tf
import std_msgs.msg

class LocalizationNode(Node):
    def __init__(self):
        # Inicializar el nodo
        super().__init__('localization_node')
        self.get_logger().info("Inicializando el nodo de localización...")
        
        # Cargar el mapa
        map_file = "mapa1.pcd"
        self.map_cloud = o3d.io.read_point_cloud(map_file)
        if self.map_cloud.is_empty():
            self.get_logger().error(f"No se pudo cargar el archivo de mapa: {map_file}")
            return
        self.get_logger().info("Archivo de mapa cargado correctamente.")
        self.initial_transformation = np.identity(4)
        
        # Crear suscriptores/publicadores
        qos_profile_reliable = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        qos_profile_best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/Odometry', # topic de la pose estimada del robot
            self.odom_callback,
            qos_profile_reliable
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points', # topic de la nube de puntos del LiDAR
            self.points_callback,
            qos_profile_best_effort
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            '/cloud', # topic en el que se publica la nube transformada segun la pose del robot
            qos_profile_best_effort
        )
        self.get_logger().info("Suscripciones y publicador configurados correctamente.")

    def odom_callback(self, msg): # Se actualizan las componentes (x,y,yaw) de la pose del robot
        trans = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        odom_translation = np.array([trans.x, trans.y, trans.z])
        z_rotation = rot.z
        w_rotation = np.sqrt(1 - z_rotation**2) 
        odom_rotation = [w_rotation, 0, 0, z_rotation]
        self.initial_transformation = self.from_transform_to_matrix(odom_translation, odom_rotation)

    def from_transform_to_matrix(self, translation, rotation): # Se crea la matriz de transformacion de la pose del robot
        transformation_matrix = tf.concatenate_matrices(
            tf.translation_matrix(translation),
            tf.quaternion_matrix(rotation)
        )
        return transformation_matrix

    def points_callback(self, msg): # Se actualiza la nube de puntos
        points = np.array([np.array((point[0], point[1], point[2])) for point in point_cloud2.read_points(msg, skip_nans=True)], dtype=np.float32)
        current_cloud = o3d.geometry.PointCloud()
        current_cloud.points = o3d.utility.Vector3dVector(points)
        current_cloud.transform(self.initial_transformation)
        self.publish_transformed_cloud(current_cloud)

    def publish_transformed_cloud(self, cloud): # Se publica la nube de puntos transformada segun la pose del robot
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'os_map'
        cloud_msg = point_cloud2.create_cloud_xyz32(header, np.asarray(cloud.points))
        self.cloud_publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
