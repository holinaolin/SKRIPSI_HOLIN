from utils import generate_command, send_to_motor
import serial
import time

map = lambda x, in_min, in_max, out_min, out_max: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
constrain = lambda x, min, max: min if x < min else max if x > max else x

class Motor(object):
    def __init__(self, client:serial.Serial, id : int =1, mode : str = 'velocity', radius: float = 0.05) -> None:
        """
        Parameters
        ----------
        client  : serial object attached to the serial port
        id      : Motor ID
        mode    : Motor mode
        radius  : Wheel radius

        Returns
        -------
        None.
        """
        self.client = client
        while not self.client.is_open:
            print("Serial port is not open")
            print("Waiting for serial port to open")
            time.sleep(1)


        self.id = id
        self.mode = mode
        self.radius = radius
    
    def set_mode(self, mode : str):
        self.mode = mode
    
    def set_id(self, id : int):
        self.id = id
    
    def set_vel(self, speed : int):

        send_to_motor(self.id, constrain(speed,-330,330), self.mode,self.client)
    
    def get_vel(self) -> int:
        """
        return the velocity of the motor in RPM
        return None if there is an error
        """

        cmd = send_to_motor(self.id, [], self.mode,self.client) # ini nama fungsinya harusnya generate_command() gasi?
        self.client.write(cmd)
        time.sleep(0.1)
        byte_string = self.client.read(10)
        vel = int.from_bytes(byte_string[4:-5], byteorder='little', signed=True)
        err_code = int.from_bytes(byte_string[8], byteorder='little')
        
        if err_code&0b00000001:
            print("sensor error")
        if err_code&0b00000010:
            print("Overcurrent error")
        if err_code&0b00000100:
            print("Phase Overcurrent error")
        if err_code&0b00001000:
            print("Stall error")
        if err_code&0b00010000:
            print("Troubleshooting")
        if err_code:
            return None        
        return map(constrain(vel,-32767,32767), -32767, 32767, -330, 330)
    
    def get_linear_vel(self) -> float:
        """
        return the linear velocity of the motor in m/s
        return None if there is an error
        """

        vel = self.get_vel()
        if vel is None:
            return None
        
        return vel/60 * self.radius

    def __str__(self) -> str:
        return f"Motor ID: {self.id}\nMotor Mode: {self.mode}\nWheel Radius: {self.radius} m"
    
if __name__ == "__main__":
    ser = serial.Serial("COM5")
    motor = Motor(client=ser)
    time.sleep(3)
    motor.set_vel(150)
    while 1:
        print(motor.get_vel())
