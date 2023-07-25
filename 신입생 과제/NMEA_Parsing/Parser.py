from Mod import Ser, Parsing, Printer

while True:
    ser = Ser()  # Serial Class
    str = ser.read()  # NMEA Raw Data
    if str[3:6] == "GGA":  # Data 선별
        par = Parsing(str)  # Parsing Class
        nmea_parse = par.parser()  # Parsing 실행
        pri = Printer(nmea_parse, str)  # Print Class
        pri.printer()  # Print 실행
    else:
        pass
