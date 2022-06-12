import time
import sqlite3 # python3 -m pip install sqlite3
import urllib3 # python3 -m pip install urllib3

http = None

keys = {
    'ldr': 'TBD',
    'tmp': 'TBD',
    'hum': 'TBD'
}

def send(data):
    api_key = keys[data[0]]

    r = http.request(
        'GET', 
        'https://api.thingspeak.com/update?api_key={}&field1={}'.format(api_key, data[1])
    )

    if r.status != 200:
        print('Some error sending data')

if __name__ == '__main__':
    con = sqlite3.connect('sedu.db')
    cur = con.cursor()

    http = urllib3.PoolManager()

    try:
        while True:
            
            for row in cur.execute('SELECT id, value, time FROM sensors'):
                if row[0] != 'cks':
                    print(row)
                    send(row)
                con.execute('delete from sensors where id="{}" and value={} and time="{}"'.format(row[0],row[1], row[2]))

            con.commit()
            time.sleep(1)
    except(KeyboardInterrupt, e):
        print('Program stopped')
        con.close()

    
    