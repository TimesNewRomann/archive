import serial
from pyproj import Transformer
import pynmea2
import rclpy
from rclpy.node import Node
from custom_interface.msg import Gpsdata
from rclpy.qos import QoSProfile
import pandas as pd
import os


class Publisher(Node):
    def __init__(self):
        super().__init__('nmea_parsing')  # 노드이름은 nmea_parsing
        qos_profile = QoSProfile(depth=10)
        self.gaa_publisher = self.create_publisher(
            Gpsdata, 'nmea_data', qos_profile)  # 토픽이름은 nmea_data
        self.timer = self.create_timer(0.1, self.publish_nmea_data)
        self.nmea = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.path = '/home/vilab/ros2_ws/coordinates.csv'

    def publish_nmea_data(self):
        if self.nmea.readable():
            string = self.nmea.readline()
            string = string.decode(
                encoding='UTF-8', errors='ignore')[:len(string)-1].strip()
            if string[3:6] == 'GGA' and len(string) >= 71:
                nmea_parse = pynmea2.parse(string)
                wgs_lat = int(nmea_parse.lat[:2]) + \
                    float(nmea_parse.lat[2:]) / 60
                wgs_lon = int(nmea_parse.lon[:3]) + \
                    float(nmea_parse.lon[3:]) / 60

                transformer = Transformer.from_crs(
                    'EPSG:4326', 'EPSG:32652')  # WGS to UTM
                utm_lat, utm_lon = transformer.transform(wgs_lat, wgs_lon)

                msg = Gpsdata()  # 메세지
                msg.raw_data = string
                msg.msg_id = string[1:6]
                msg.utc = string[7:13]
                msg.lat = wgs_lat
                msg.lat_dir = nmea_parse.lat_dir
                msg.lon = wgs_lon
                msg.lon_dir = nmea_parse.lon_dir
                msg.quality = nmea_parse.gps_qual
                msg.satell = int(nmea_parse.num_sats)
                msg.hdop = float(nmea_parse.horizontal_dil)
                msg.alt = nmea_parse.altitude
                msg.alt_unit = nmea_parse.altitude_units
                msg.sep = float(nmea_parse.geo_sep)
                msg.sep_unit = nmea_parse.geo_sep_units
                msg.diff_age = nmea_parse.age_gps_data
                msg.diff_station = nmea_parse.ref_station_id
                msg.check_sum = str(string[-2:])  # .rstrip()
                msg.utm_lat = utm_lat
                msg.utm_lon = utm_lon
                # df = pd.read_csv('/home/seonwoo/ros2_ws/coordinates.csv', sep=',',
                #  encoding='cp949')  # 작업 데이터프레임
                data = {'UTM_lat': utm_lat, 'UTM_lon': utm_lon}
                # df = pd.concat(
                # [df, pd.DataFrame(data, index=[0])], ignore_index=True)
                # df.to_csv('/home/seonwoo/ros2_ws/coordinates.csv',
                # index=False, encoding='cp949')
                df = pd.DataFrame(data, index=[0])

                if not os.path.exists(self.path):
                    df.to_csv(self.path, index=False, encoding='cp949')
                else:
                    df.to_csv(self.path, mode='a', header=False,
                              index=False, encoding='cp949')
                self.gaa_publisher.publish(msg)
                self.get_logger().info('Published NMEA Data')
            else:
                self.get_logger().info('Invalid NMEA data, ignoring')


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
