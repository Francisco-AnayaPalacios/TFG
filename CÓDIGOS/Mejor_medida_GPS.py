import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path, Odometry
import pyproj
import csv
import os
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from time import time

qos_profile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT)

class GpsSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.start_time = None
        self.subscription_gps = self.create_subscription(
            NavSatFix,
            '/hunter/fix',
            self.gps_callback,
            10)
        self.subscription_path = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10)
        self.subscription_imu = self.create_subscription(
            Imu,
            #'/ouster/imu',
            '/mti/imu',
            self.imu_callback,
            qos_profile)

        # UTILIZANDO VALORES INICIALES NONE
        #self.gps_coords = None
        #self.orientation = None
        #self.latest_path_coords = None
        #self.latest_odom_coords = None
        #self.gps_received = False
        #self.path_received = False
        #self.odom_received = False
        #self.imu_received = False
        #self.min_cov_x = None
        #self.min_cov_y = None
        #self.min_cov_z = None
        
        # UTILIZANDO VALORES POR DEFECTO
        self.gps_coords = None
        self.orientation = (0, 0, 0, 0)      
        self.latest_path_coords = (0,0,0)
        self.latest_odom_coords = (0,0,0)
        self.gps_received = False
        self.path_received = False
        self.odom_received = False
        self.imu_received = False
        self.min_cov_x = None
        self.min_cov_y = None
        self.min_cov_z = None
        self.cov_threshold = 10000.0  

        self.crs_wgs = pyproj.CRS("EPSG:4326")  
        self.crs_utm = pyproj.CRS("EPSG:25830")  

        self.transformer = pyproj.Transformer.from_crs(self.crs_wgs, self.crs_utm, always_xy=True)

        self.csv_file = os.path.join(os.path.dirname(__file__), 'georeference/georeference.csv')
        self.csv_header_written = False 
        self.write_csv_header()

    def write_csv_header(self):
        if not self.csv_header_written:
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file, delimiter=';')
                writer.writerow([
                    'x_gps_utm', 'y_gps_utm', 'z_gps', 
                    'x_path', 'x_path', 'z_path', 
                    'x_odom', 'y_odom', 'z_odom', 
                    'orient_x', 'orient_y', 'orient_z', 'orient_w', 
                    'cov_x', 'cov_y', 'cov_z', 'time_elapsed'])
            self.csv_header_written = True

    def gps_callback(self, msg):
        self.gps_received = True
        print("Mensaje recibido por /hunter/fix")
        if self.start_time is None:
            self.start_time = time()

        if self.min_cov_x is None or msg.position_covariance[0] < self.min_cov_x:
            print(f"La covarianza actual es la menor: {msg.position_covariance[0]}")
            latitud = msg.latitude
            longitud = msg.longitude
            altitud = msg.altitude
            x, y = self.transformer.transform(longitud, latitud)
            print(f'Coordenadas GPS: Latitud {latitud}, Longitud {longitud}, Altitud {altitud}')
            print(f'Coordenadas UTM: X {x}, Y {y}')

            self.min_cov_x = msg.position_covariance[0]
            self.min_cov_y = msg.position_covariance[4]
            self.min_cov_z = msg.position_covariance[8]
            self.gps_coords = (x, y, altitud)

            # Actualizar los valores solo si la cov_x es menor
            self.update_values()
        else:
            print(f"La covarianza actual es NO la menor: {msg.position_covariance[0]}")
            print(f"La menor es: {self.min_cov_x}")

    def path_callback(self, msg):
        if msg.poses:
            self.path_received = True
            posicion = msg.poses[-1].pose.position
            self.latest_path_coords = (posicion.x, posicion.y, posicion.z)


    def odom_callback(self, msg):
        self.odom_received = True
        posicion = msg.pose.pose.position
        self.latest_odom_coords = (posicion.x, posicion.y, posicion.z)


    def imu_callback(self, msg):
        self.imu_received = True
        orientacion_q = msg.orientation
        self.orientation = (orientacion_q.x, orientacion_q.y, orientacion_q.z, orientacion_q.w)


    def update_values(self):
        if self.gps_coords and self.orientation and self.csv_header_written:
            time_elapsed = time() - self.start_time
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file, delimiter=';')
                # Comprobar si hay valores vacíos en las coordenadas antes de escribir la fila
                if None not in [self.gps_coords, self.latest_path_coords, self.latest_odom_coords, self.orientation]:
                    writer.writerow([
                        self.gps_coords[0], self.gps_coords[1], self.gps_coords[2], 
                        self.latest_path_coords[0] if self.latest_path_coords else "", self.latest_path_coords[1] if self.latest_path_coords else "", self.latest_path_coords[2] if self.latest_path_coords else "",
                        self.latest_odom_coords[0] if self.latest_odom_coords else "", self.latest_odom_coords[1] if self.latest_odom_coords else "", self.latest_odom_coords[2] if self.latest_odom_coords else "",
                        self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3],
                        self.min_cov_x, self.min_cov_y, self.min_cov_z, time_elapsed
                    ])
                    print(f'Datos guardados en {self.csv_file}')
                else:
                    print('No se escribió la fila en el archivo CSV debido a valores vacíos en las coordenadas.')
        else:
            print('Esperando información completa de GPS, IMU y trayectoria.')

def main(args=None):
    rclpy.init(args=args)
    node = GpsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
