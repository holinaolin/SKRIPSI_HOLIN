
"""
    Kode ini dijalankan pada robot AGV
"""
import socket
import json
from ddsm115 import MotorControl
import struct
import pickle
import time
from threading import Thread, Lock

def receive_data(conn):
    """
    @brief: receive data from the connection assuming that
        first 4 bytes represents data size,
        next 4 bytes represents data identifier and
        successive bytes of the size 'data size'is payload
    @args[in]:
        conn: socket object for conection from which data is supposed to be received
    """
    
    # receive first 4 bytes of data as data size of payload

    # try:
    recv = conn.recv(4)
    if len(recv) == 0:
        print("no payload received")
        return None
    data_size = struct.unpack(">I",recv)[0]
    # except:
    #     return None
    # receive next 4 bytes of data as data identifier
    # data_id = struct.unpack(">I", conn.recv(4))[0]
    # receive payload till received payload size is equal to data_size received
    received_payload = b""
    reamining_payload_size = data_size
    while reamining_payload_size != 0:
        received_payload += conn.recv(reamining_payload_size)
        reamining_payload_size = data_size - len(received_payload)
    payload = pickle.loads(received_payload)
    return  payload

def send_data(data):
    global s
    # print("Data dikirim: ", data)
    serialized_payload = pickle.dumps(data)
    # send data size, data identifier and payload
    s.sendall(struct.pack(">I", len(serialized_payload)))
    # s.sendall(struct.pack(">I", data_id))
    s.sendall(serialized_payload)

def update():
    global motki, motka
    while True:
        with motki_lock:
            vel1, cur1 = motki.get_feedback(1)
        with motka_lock:
            vel2, cur2 = motka.get_feedback(1)
        data = {"arus_kiri":cur1,"rpm_kiri":vel1, "arus_kanan":cur2, "rpm_kanan":vel2}
        send_data(data)
        time.sleep(1)

if __name__ == "__main__":
    # Inisialisasi socket server
    motki = MotorControl(device="COM3")
    motka = MotorControl(device="COM5")
	
	# motor.set_id(1)
    motki.set_drive_mode(_id=1, _mode=2)
    motka.set_drive_mode(_id=1, _mode=2)
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Bind ke alamat localhost (ip server) dengan port 12345
    s.connect(('192.168.18.94', 12345))
    s.settimeout(30)

    motka_lock = Lock()
    motki_lock = Lock()
    
    while True:
        # print("Connected by: ", addr)
        # while(True):
        # Listen ke koneksi
        
        data = receive_data(s)

        if data is None:
            continue
        print("Data diterima: ", data)
        data = json.loads(data)
        print(data)
        
        if(data["finish"]):
            break
        print("motor kiri: ", data["kiri"])
        print("motor kanan: ", data["kanan"])
        # time.sleep(1)
        with motki_lock:
            motki.send_rpm(1, data["kiri"])
        with motka_lock:
            motka.send_rpm(1, data["kanan"])    

        