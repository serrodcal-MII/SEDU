import serial, time

if __name__ == '__main__':
    
    micro = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    
    while True:
        if (micro.isOpen()):
            micro.write(b'[S]')
            line = micro.readline()
            line = line.decode('utf-8').lstrip('[O').rstrip(']\r\n')
            print(line)
            
        
        time.sleep(2)