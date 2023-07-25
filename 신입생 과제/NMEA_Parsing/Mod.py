import serial
from pyproj import Transformer
import pynmea2


class Ser:  # 시리얼 통신
    def __init__(self):
        self.nmea = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def read(self):
        if self.nmea.readable():
            string = self.nmea.readline()
            string = string.decode(encoding='UTF-8',
                                   errors='ignore')[:len(string)-1]
            return string  # Raw Data


class Parsing:
    def __init__(self, str):
        self.str = str

    def parser(self):
        nmea_parse = pynmea2.parse(self.str)  # Data 해석
        return nmea_parse


class Printer:
    def __init__(self, nmea_parse, str):
        self.nmea_parse = nmea_parse
        self.str = str

    def printer(self):
        # NMEA to WGS
        wgs_lat = int(self.nmea_parse.lat[:2]) + \
            (float(self.nmea_parse.lat[2:]) / 60)
        # NMEA to WGS
        wgs_lon = int(self.nmea_parse.lon[:3]) + \
            (float(self.nmea_parse.lon[3:]) / 60)
        transformer = Transformer.from_crs(
            'EPSG:4326', 'EPSG:32652')  # WGS to UTM
        utm_lat, utm_lon = transformer.transform(
            wgs_lat, wgs_lon)  # WGS to UTM 실행

        # Data 시각화및 수치화
        print('-'*10, 'GGA DATA', '-' * 10)
        print(f'GGA.raw_data : {self.nmea_parse}')
        print(f'GGA.message_id : {self.nmea_parse.timestamp}')
        print(f'GGA.lat : {wgs_lat}')
        print(f'GGA.lat_dir : {self.nmea_parse.lat_dir}')
        print(f'GGA.lon : {wgs_lon}')
        print(f'GGA.lon_dir : {self.nmea_parse.lon_dir}')
        print(f'GGA.quality : {self.nmea_parse.gps_qual}')
        print(f'GGA.num_satellites Used : {self.nmea_parse.num_sats}')
        print(f'GGA.HDOP : {self.nmea_parse.horizontal_dil}')
        print(f'GGA.alt : {self.nmea_parse.altitude}')
        print(f'GGA.alt_unit : {self.nmea_parse.altitude_units}')
        print(f'GGA.sep : {self.nmea_parse.geo_sep}')
        print(f'GGA.sep_unit : {self.nmea_parse.geo_sep_units}')
        print(f'GGA.diff_age : {self.nmea_parse.age_gps_data}')
        print(f'GGA.diff_station : {self.nmea_parse.ref_station_id}')
        print(f'GGA.check_sum : {self.str[-3:]}')
        print(f'GGA.utm_lat : {utm_lat}')  # 안양(316043.9593771873)
        print(f'GGA.utm_lon : {utm_lon}')  # 안양(4140951.9577732906)
        print(f'------------------------------')
