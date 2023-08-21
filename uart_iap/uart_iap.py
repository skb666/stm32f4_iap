if __name__ == "__main__":
    import struct
    import time
    from mySerial import *
    
    portname = "COM5"
    baudrate = 921600
    filename = "../app/emStudio/Output/Release/Exe/app.bin"
    readsize = 1024
    
    ports = SerialCommand.get_all_ports()
    if not portname in ports:
        print(ports)
        exit()
        
    sercomm = SerialCommand(portname, baudrate)
    if sercomm.openserial() < 0:
        print(f"Can't Open `{portname}`")
        exit()
    sercomm.used_port_info()
    
    packed = b"\x55\xaa" + struct.pack('<H', 0xfffe)[:1]
    chars = sercomm.write_raw(packed)
    print(f"send {chars} chars")
    time.sleep(0.1)
    
    packed = b"\x55\xaa\x01" + struct.pack('<H', 0)
    chars = sercomm.write_raw(packed)
    print(f"send {chars} chars")
    time.sleep(0.1)

    has_err = False
    with open(filename, "rb") as f_obj:
        while True:
            data = f_obj.read(readsize)
            if not data:
                break
            data_len = len(data)
            if data_len != readsize:
                data += b'\xff' * (4 - data_len % 4)
                data_len = len(data)
            packed = b"\x55\xaa\x00" + struct.pack('<H', data_len) + data
            chars = sercomm.write_raw(packed)
            print(f"send {chars} chars")
            recv = sercomm.read_raw(9)
            print(recv)
            if recv != b"trans ok\n":
                has_err = True
                break
    
    if not has_err:
        packed = b"\x55\xaa\x02" + struct.pack('<H', 0)
        chars = sercomm.write_raw(packed)
        print(f"send {chars} chars")
    
    print("start recv")
    sr = SerialReceive(sercomm)
    sr.start()
        
    while True:
        sdata = input()
        if sdata == "q!":
            break
 
    sercomm.closeserial()