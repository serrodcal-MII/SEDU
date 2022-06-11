Configurar script de Python como servicio al arranque del sistema:

1. [Opcional] Instalar systemd si no está instalado: sudo apt install -y systemd

2. Comprobar que systemd está instalado: systemd --version

3. Crear el script de Python: sudo nano pasarela.py

4. Crear el servicio: sudo nano /etc/systemd/system/pasarela.service

[Unit]
Description=Servicio de pasarela
After=multi-user.target

[Service]
Type=simple
Restart=always
ExecStart=/usr/bin/python3 /home/serrodcal/pasarela.py

[Install]
WantedBy=multi-user.target

5. Recargamos el demonio: sudo systemctl daemon-reload

6 Habilitamos nuestro servicio para que no se deshabilite si el servidor se reinicia.: sudo systemctl enable pasarela.service

7. Arrancamos el servicio: sudo systemctl start pasarela.service

8. La base de datos se crea en /, como sedu.db, pero no está inicializada. Paramos el servicio con: sudo systemctl stop pasarela.service

9. Copiamos el script de creación de tabla en /: sudo cp ~/Desktop/modulo7/create_db.py

10. Ejecutamos la creación de tabla: sudo python3 create_db.py

11. Iniciamos el servicio de nuevo: sudo systemctl start pasarela.service

12. Comprobamos que está escribiendo entradas en la base de datos (en /):

> sudo sqlite3 sedu.db
>> select * from sensors;