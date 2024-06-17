import crcmod

# def senses(client, slave, fn_cod=0):
#     bytes = b''
#     out = client.read()
#     if out == slave:
#         bytes = 

def extract_sensors_value(serial_client, slave, fn_code=3):
    byte_string = serial_client.read(9)
    print(byte_string)
    if  byte_string != b'' and byte_string[0] == slave and byte_string[1] == fn_code:
        value = int.from_bytes(byte_string[3:-4], byteorder='big')
        return conversion_magnetic(value)

def conversion_magnetic(value):
    median = (value-256)*0.00390625+1
    if median % 1 != 0:
        median = round(median + 0.5, 1)
    return median

def map_value(inp_val, inp_min, inp_max, out_min, out_max):
    out_val = int((inp_val-inp_min)/(inp_max-inp_min) * (out_max-out_min) + out_min)
    return out_val

def twos_complement(value, num_bits):
    if value < 0:
        positive_value = value.to_bytes(int(num_bits/8), 'big', signed=True)
    else:
        positive_value = value.to_bytes(int(num_bits/8), 'big')
    return positive_value

def send_to_motor(id, value, mode, client):
    crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')
    command = command_conversion(id, value, mode=mode)
    checksum = crc8(command).to_bytes(1, 'big')
    command_bytes = command + checksum
    client.write(command_bytes)

def command_conversion(id:int, value:int, mode:str, bits=16):
    if mode.lower() == "current" :
        value = map_value(value, -8, 8, -32767, 32767)
    elif mode.lower() == "position" :
        value = map_value(value, 0, 360, 0, 32767)
    elif mode.lower() == "velocity" :
        value = value
    value = twos_complement(value, num_bits=bits)
    command = id.to_bytes(1, 'big') + b'd%b\x00\x00\x00\x00\x00'%value
    return command

def generate_command(command:str):
    crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')
    byte_string = bytes.fromhex(command)
    checksum = crc8(byte_string).to_bytes(1, 'big')
    command_bytes = byte_string + checksum
    return command_bytes


def get_feedback_rpm(id, client):
    command = f"{id:02d} 74 00 00 00 00 00 00 00"
    command = generate_command(command)
    client.write(command)
    v_fb = int.from_bytes(client.read(10)[4:6], byteorder='little')
    return v_fb