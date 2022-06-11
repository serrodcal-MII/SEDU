import sqlite3

con = sqlite3.connect('sedu.db')
cur = con.cursor()

cur.execute('CREATE TABLE sensors (id text, value int, time timestamp)')

con.commit()
con.close()