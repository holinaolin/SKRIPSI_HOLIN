import socket 
import threading
import time
import json
import pickle
import struct

class PID:
    def __init__(self, kp = 0, ki = 0, kd = 0, max_output = 50, integral_saturation = 0):
        self.error = 0
        self.integral = 0
        self.prev_error = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral_saturation = integral_saturation
        
    def update(self, setpoint, current):
        self.error = setpoint - current
        p_term = self.error * self.kp
        integral = self.integral + self.error
        if self.integral_saturation != 0:
            if integral > self.integral_saturation:
                integral = self.integral_saturation
            elif integral < -self.integral_saturation:
                integral = -self.integral_saturation
        i_term =  integral * self.ki
        d_term = (self.error - self.prev_error) * self.kd
        self.prev_error = self.error
        return max(-self.max_output,min(self.max_output,(p_term + i_term + d_term)))

    def __str__(self, pre : str = "PID CLASS") -> str:
        data = pre+f"""
kp: {self.kp}
ki: {self.ki}
kd: {self.kd}
max_output: {self.max_output}
integral_saturation: {self.integral_saturation}

current error: {self.error}
previous error: {self.prev_error}
current integral value: {self.integral}
"""
        return data

class PIDHeading(PID):
    def __init__(self,kp = 0, ki = 0, kd = 0, max_output = 30, integral_saturation = 0):
        super().__init__(kp, ki, kd, max_output, integral_saturation)
        
    def update(self, setpoint, current):
        error = setpoint - current
        if(error > 180):
            error = error - 360
        elif(error < -180):
            error = error + 360
        
        self.error = error
        p_term = self.error * self.kp
        self.integral = self.integral + self.error
        if self.integral_saturation != 0:
            if self.integral > self.integral_saturation:
                self.integral = self.integral_saturation
            elif self.integral < -self.integral_saturation:
                self.integral = -self.integral_saturation
        i_term =  self.integral * self.ki
        d_term = (self.error - self.prev_error) * self.kd
        self.prev_error = self.error
        return -max(-self.max_output,min(self.max_output,(p_term + i_term + d_term)))

    def __str__(self, pre : str = "PIDHeading CLASS") -> str:
        return super().__str__(pre)

class Robot:
    connected = False
    def __init__(self, address, port, max_rpm=330):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # read own ip address
        host_name = socket.gethostname()
        self.ip_address = socket.gethostbyname(host_name)
        if address != "":
            self.ip_address = address
        print(f"IP address: {self.ip_address}")
        print(f"host name: {host_name}")

        self.server.bind((self.ip_address, port))
        self.server.listen()
        self.conn , self.client_addr = self.server.accept()
        self.connected = True
        self.rpm = [0,0]
        t_main = threading.Thread(target=self.main)
        t_main.start()

        self.max_rpm = max_rpm
        self.finish_state = False

        self.arus_kiri = 0
        self.rpm_kiri = 0
        self.arus_kanan = 0
        self.rpm_kanan = 0
        t_update = threading.Thread(target=self.update)
        t_update.start()
        time.sleep(3)

    def receive_data(self):
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
        recv = self.conn.recv(4)
        if len(recv) == 0:
            print("no payload received")
            return None
        data_size = struct.unpack(">I",recv)[0]
        # except:
        #     return None
        # receive next 4 bytes of data as data identifier
        # data_id = struct.unpack(">I", self.conn.recv(4))[0]
        # receive payload till received payload size is equal to data_size received
        received_payload = b""
        reamining_payload_size = data_size
        while reamining_payload_size != 0:
            received_payload += self.conn.recv(reamining_payload_size)
            reamining_payload_size = data_size - len(received_payload)
        payload = pickle.loads(received_payload)
        # print(payload)
        return  payload

    def update(self):
        while True:
            data = self.receive_data()
            # print("Data diterima: ", data)
            if data is not None:
                data_dict = json.loads(data)
                self.arus_kiri = data_dict["arus_kiri"]
                self.rpm_kiri = data_dict["rpm_kiri"]
                self.arus_kanan = data_dict["arus_kanan"]
                self.rpm_kanan = data_dict["rpm_kanan"]
            time.sleep(0.1)

    def get_motor_current(self):
        return self.arus_kiri, self.arus_kanan

    def get_motor_rpm(self):
        return self.rpm_kiri, self.rpm_kanan

    def finish(self):
        self.finish_state = True

    def main(self):
        with self.conn:
            print("Connected by: ", self.client_addr)
            counter = 0
            while True:
                # print("counter: ", counter)
                counter += 1

                data = {"kiri":self.rpm[0], "kanan":self.rpm[1], "finish":self.finish_state}

                data = json.dumps(data)
                # print(data)

                self.send_data(data)
                time.sleep(1)

                if self.finish_state:
                    break

    def send_data(self, data):
        # print("Data dikirim: ", data)
        serialized_payload = pickle.dumps(data)
        # send data size, data identifier and payload
        self.conn.sendall(struct.pack(">I", len(serialized_payload)))
        # conn.sendall(struct.pack(">I", data_id))
        self.conn.sendall(serialized_payload)
        # print("Data terkirim")

    def set_rpm(self, rpm):
        if rpm[0] > self.max_rpm:
            rpm[0] = self.max_rpm
        elif rpm[0] < -self.max_rpm:
            rpm[0] = -self.max_rpm

        if rpm[1] > self.max_rpm:
            rpm[1] = self.max_rpm
        elif rpm[1] < -self.max_rpm:
            rpm[1] = -self.max_rpm

        self.rpm = rpm


class RobotDummy:
    def get_motor_current(self):
        return 0,0

    def get_motor_rpm(self):
        return 0,0

    def finish(self):
        pass

    def main(self):
        pass

    def set_rpm(self, rpm):
        print("set rpm", rpm)

if __name__ == "__main__":
    # from simul_control import DifferentialDriveSim
    from math import sqrt, degrees, radians, atan2
    import matplotlib.pyplot as plt
    import numpy as np
    # robot = Robot("127.0.0.1", 12345)
    # while not robot.connected:
    #     pass

    pid = PID(kp=0.2, ki=0.08, kd=0, max_output=80, integral_saturation=2)
    pid_heading = PIDHeading(kp=10, ki=1, kd=0, max_output=50, integral_saturation=0)
    x = 0
    y = 2
    theta = radians(0)
    # # sim = DifferentialDriveSim(x=x, y=x, theta=theta, wheelbase=0.24, wheel_radius=0.115, max_speed=0.5)

    # node = [[1,1], [3,3]]
    # # node_counter = 0
    # x_tujuan = 1
    # y_tujuan = 1

    # jarak = sqrt((x_tujuan-x)**2 + (y_tujuan-y)**2)
    # odom = []

    # plt.ion()
    # plt.xlabel("X-axis")
    # plt.ylabel("Y-axis")
    # figure, ax = plt.subplots(figsize=(10, 8))
    # ax.set_xlim(-1, 5)
    # ax.set_ylim(-1, 5)
    # x, y, theta = sim.get_pose()
    # odom.append([x, y, theta])
    # print(f"odom {odom}")
    # odom_fig = ax.plot([], [], 'r-')[0]
    # # figure.canvas.draw()
    # for x_tujuan, y_tujuan in node:
    #     while True:
    #         x, y, theta = sim.get_pose()
    #         jarak = sqrt((x_tujuan-x)**2 + (y_tujuan-y)**2)
    #         theta_tujuan = atan2(y_tujuan-y, x_tujuan-x)
    #         print(f"jarak: {jarak}, theta_tujuan: {degrees(theta_tujuan)}, theta: {degrees(theta)}")
    #         print(f"error: {jarak}, heading error: {degrees(theta_tujuan)-degrees(theta)}")
    #         v = pid.update(0, -jarak)
    #         w = pid_heading.update(degrees(theta_tujuan), degrees(theta))
    #         print(f"v: {v}, w: {w}")
    #         v_l, v_r = sim.get_wheel_speeds(v, radians(w))
    #         print(f"v_l: {v_l}, v_r: {v_r}")
    #         rpm_l = max(-50,min(50, v_l * 60 / (2 * 3.14 * 0.115)))

    #         rpm_r = max(-50,min(50,v_r * 60 / (2 * 3.14 * 0.115)))
    #         print(f"rpm_l: {rpm_l}, rpm_r: {rpm_r}")
    #         sim.update_rpm(rpm_l=rpm_l,rpm_r=rpm_r ,dt= 0.1)
    #         # sim.update(v_l, v_r, 0.1)
    #         # robot.set_rpm([v_l, v_r])
    #         x, y, theta = sim.get_pose()
    #         print(f"x: {round(x,3)}, y: {round(y,3)}, theta: {degrees(round(theta,3))}")
    #         odom.append([x, y, theta])
    #         # print(f"odom {odom}")
    #         # print(f"odom[:,0] {np.array(odom)[:,0]}")
    #         odom_fig.set_xdata(np.array(odom)[:,0])
    #         odom_fig.set_ydata(np.array(odom)[:,1])
    #         figure.canvas.draw()
    #         figure.canvas.flush_events()
    #         time.sleep(0.1)
    #         if jarak < 0.1:
    #             break

    # print("finish")
    # robot.finish()
    
    




    # robot.set_rpm([10,10])
    # time.sleep(10)
    # robot.set_rpm([50,50])
    # time.sleep(10)
    # print("finish")
    # robot.finish()
    # pid = PID(kp=0.1, kd=0.02)
    # setpooint = 10
    # current = 5
    # value = pid.update(setpooint, current)
    
    # robot.set_rpm([value, value])

    # print(pid)
    # pid_heading = PIDHeading(kp=1,integral_saturation=10)
    # print(pid_heading)
