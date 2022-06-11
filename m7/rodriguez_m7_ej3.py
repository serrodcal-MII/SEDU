import serial, time

def checksum(sensors):
    ldr = int(sensors['ldr'])
    tmp = int(sensors['tmp'])
    hum = int(sensors['hum'])
    cks = int(sensors['cks'])
    
    return cks == ldr + tmp + hum
    

if __name__ == '__main__':
    
    micro = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    
    while True:
        if (micro.isOpen()):
            micro.write(b'[S]')
            line = micro.readline()
            line = line.decode('utf-8').lstrip('[O').rstrip(']\r\n')
            
            values = line.split(',')
            sensors = dict(zip(['ldr','tmp', 'hum', 'cks'], values))
            
            res = checksum(sensors)
            
            print('Checksum check:', res)
            print('Values: ldr={}, temperature={}, humidity={}'.format(int(sensors['ldr']),int(sensors['tmp']),int(sensors['hum'])))
        
        time.sleep(5)
