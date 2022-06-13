import serial, time, sqlite3

def checksum(sensors):
    ldr = int(sensors['ldr'])
    tmp = int(sensors['tmp'])
    hum = int(sensors['hum'])
    cks = int(sensors['cks'])
    
    return cks == ldr + tmp + hum

def run_query(query=''):
    con = sqlite3.connect('sedu.db')
    cur = con.cursor()

    cur.execute(query)

    con.commit()
    con.close()
    
    return

def bind_query(sensors):
    for k,v in sensors.items():
        query = f'INSERT INTO sensors VALUES ("{k}", {v}, "{time.strftime("%Y-%m-%d %H:%M:%S")}")'
        run_query(query)
    return

if __name__ == '__main__':
    
    micro = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)
    
    try:
        while True:
            if (micro.isOpen()):
                micro.write(b'[S]')
                line = micro.readline()
                line = line.decode('utf-8').lstrip('[O').rstrip(']\r\n')
                
                values = line.split(',')
                sensors = dict(zip(['ldr','tmp', 'hum', 'cks'], values))
                
                if checksum(sensors):
                    bind_query(sensors)
            
            print('Sensor running...')
            time.sleep(5)
    except(KeyboardInterrupt, e):
        print('Program stopped')
        micro.close()


 
