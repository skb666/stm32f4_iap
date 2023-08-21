import serial
import threading
 
class SerialCommand:
    """
            波特率可以使用以下列表中的数值
    """
    baudrates = [9600, 38400, 115200, 230400, 460800, 921600, 1000000, 1500000]
 
    @classmethod
    def get_all_ports(cls):
        """
        类方法：获取系统上所有串口
        :return: 字符串表示的可用串口的列表
        """
        ports = []
        import serial.tools.list_ports
        ports_list = list(serial.tools.list_ports.comports())
 
        if len(ports_list) == 0:
            return None
        for i in range(0, len(ports_list)):
            port = ports_list[i]
            port = (str(port)).split(' ')[0]
            ports.append(port)
        return ports
 
    def __init__(self, portx, bps=9600, timeout=None, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                 stopbites=serial.STOPBITS_ONE):
        """
        设置使用的串口和波特率，已经超时时间
        :param portx: 要使用的串口
        :param bps:  要使用的波特率
        :param timeout: 超时时间
        :param bytesize: 字节长度
        :param parity: 校验方式
        :param stopbites: 停止位长度
        """
        self.portx = portx
        self.bps = bps
        self.timeout = timeout
        self.bytesize = bytesize
        self.parity = parity
        self.stopbites = stopbites
 
    @property
    def portx(self):
        return self.__portx
 
    @portx.setter
    def portx(self, portx):
        if portx not in SerialCommand.get_all_ports():
            raise ValueError("Use SerialCommand. get_all_ports() method to see all usable serial ports")
        else:
            self.__portx = portx
 
    @property
    def bps(self):
        return self.__bps
 
    @bps.setter
    def bps(self, bps):
        if bps not in SerialCommand.baudrates:
            raise ValueError("bps must be one of the list %s" % str(SerialCommand.baudrates))
        else:
            self.__bps = bps
 
    @property
    def timeout(self):
        return self.__timeout
 
    @timeout.setter
    def timeout(self, timeout):
        if timeout == None or timeout >= 0:
            self.__timeout = timeout
        else:
            raise TypeError("timeout should be large than 0 or None")
 
    @property
    def bytesize(self):
        return self.__bytesize
 
    @bytesize.setter
    def bytesize(self, bytesize):
        if bytesize in [serial.EIGHTBITS, serial.FIVEBITS, serial.SEVENBITS, serial.SIXBITS]:
            self.__bytesize = bytesize
        else:
            raise ValueError(
                "bytesize should be in [serial.EIGHTBITS, serial.FIVEBITS, serial.SEVENBITS, serial.SIXBITS]")
 
    @property
    def parity(self):
        return self.__parity
 
    @parity.setter
    def parity(self, parity):
        if parity in [serial.PARITY_NONE, serial.PARITY_ODD, serial.PARITY_EVEN]:
            self.__parity = parity
        else:
            raise ValueError("parity should be in [serial.PARITY_NONE,serial.PARITY_ODD,serial.PARITY_EVEN]")
 
    @property
    def stopbites(self):
        return self.__stopbites
 
    @stopbites.setter
    def stopbites(self, stopbites):
        if stopbites in [serial.STOPBITS_ONE, serial.STOPBITS_TWO, serial.STOPBITS_ONE_POINT_FIVE]:
            self.__stopbites = stopbites
        else:
            raise ValueError(
                "stopbites should in  [serial.STOPBITS_ONE,serial.STOPBITS_TWO,serial.STOPBITS_ONE_POINT_FIVE]")
 
    def openserial(self):
        """
        使用初始化时的端口名，波特率和超时时间打开一个串口
        :return: 0表示打开成功，-1表示打开失败
        """
        try:
            self.__serial = serial.Serial(self.portx, baudrate=self.bps, timeout=self.timeout, parity=self.parity,
                                          bytesize=self.bytesize, stopbits=self.stopbites)
        except:
            return -1
 
        return 0
 
    def closeserial(self):
        """
        关闭串口
        :return:
        """
        if self.__serial is not None and self.__serial.isOpen():
            self.__serial.close()
 
    def write(self, data):
        """
        通过串口发送数据
        :param data: 要发送的数据
        :return: 返回发送的字节数
        """
        if not isinstance(data, list):
            print("data should be a list instance")
            return -1
 
        for ch in data:
            ch = ch & 0x00FF
            self.__serial.write(chr(ch).encode('ascii'))
 
        return len(data)
        
    def write_raw(self, data):
        return self.__serial.write(data)
 
    def read(self, len):
        lst = []
        for i in range(len):
            if self.__serial.readable():
                ch = self.__serial.read()
                lst.append(ch)
            else:
                break
        return lst
        
    def read_raw(self, len):
        return self.__serial.read(len)
 
    def isOpen(self):
        if self.__serial is not None and self.__serial.isOpen():
            return True
        else:
            return False
 
    def used_port_info(self):
        """
        输出串口信息
        :return: None
        """
        if self.__serial is not None:
            print("Serial Port info:")
            print("Port name：", self.__serial.name)
            # print("Port：",self.__serial.port)
            print("Open:", self.__serial.isOpen())
            print("Baudrate：", self.__serial.baudrate)
            print("Bytesize", self.__serial.bytesize)
            print("Parity", self.__serial.parity)
            print("stopbits", self.__serial.stopbits)
            print("Timeout：", self.__serial.timeout)
            print("writeTimeout", self.__serial.writeTimeout)
            print("xonxoff", self.__serial.xonxoff)
            print("rtscts", self.__serial.rtscts)
            print("dsrdtr", self.__serial.dsrdtr)
            print("interCharTimeout", self.__serial.interCharTimeout)
 
 
class SerialReceive(threading.Thread):
    """
    这个类的实例用于开启一个线程，专用于接收
    """
 
    def __init__(self, serialcommand):
        threading.Thread.__init__(self, name="SerialReceive")
        self.serialcommand = serialcommand
 
    @property
    def serialcommand(self):
        return self.__serialcommand
 
    @serialcommand.setter
    def serialcommand(self, serialcommand):
        if isinstance(serialcommand, SerialCommand):
            self.__serialcommand = serialcommand
        else:
            raise TypeError("serialcommand should be a instance of class SerialCommand")
 
    def __check(self):
        if self.serialcommand.isOpen():
            return True
        else:
            return False
 
    def receive(self, strlen=1):
        lst = self.serialcommand.read(strlen)
        for ch in lst:
            print(ch.decode('utf-8'), end='')
 
    def run(self):
        while self.__check():
            try:
                self.receive(1)
            except:
                print("stop receive")
                break

 
if __name__ == "__main__":
    ports = SerialCommand.get_all_ports()
    print(ports)
    port_num, baudrate = map(int, input().split())
 
    sercomm = SerialCommand(ports[port_num], baudrate)
    sercomm.openserial()
    sercomm.used_port_info()
 
    sr = SerialReceive(sercomm)
    sr.start()
 
    while True:
        sdata = input("请输入ascii字符串： ")
        if sdata == "q!":
            break
        lst = []
 
        print("data:%s,len:%d" % (sdata, len(sdata)))
 
        length = len(sdata)
        for i in range(length):
            lst.append(ord(sdata[i]))
        print(lst)
 
        chars = sercomm.write(lst)
        print("send %d chars" % chars)
 
    sercomm.closeserial()
