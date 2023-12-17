import RPi.GPIO as GPIO
import serial

class RS485_RTU_Client:
    PORTS_DEFAULTS = ((None, 10), ('/dev/TTYAMA3', 27), ('/dev/TTYAMA4', 7), ('/dev/TTYAMA0', 21))
    other_clients = [[],[],[]] #ports, devs, f_c ports
    def __init__(self, port_no: int, dev = None, flow_control_port: int = None, **kwargs):
        # Assign default values
        if dev == None:
            if len(RS485_RTU_Client.PORTS_DEFAULTS)-1 < port_no:
                #TODO: add exception
                pass
            dev = RS485_RTU_Client.PORTS_DEFAULTS[port_no][0]
            if dev == None:
                #TODO: add exception
                pass
        if flow_control_port == None:
            if len(RS485_RTU_Client.PORTS_DEFAULTS)-1 < port_no:
                #TODO: add exception
                pass
            flow_control_port = RS485_RTU_Client.PORTS_DEFAULTS[port_no][1]
            if flow_control_port == None:
                #TODO: add exception
                pass

        self.port_no = port_no
        self.dev = dev
        self.flow_control_port = flow_control_port

        # TODO: fault control
        if self.__check_overlap():
            #TODO: add except
            pass
        self.__configure_fc_port()
        self.__configure_connection()
        # TODO: adding client ot static list
        for index, value in enumerate(self.port_no, self.dev, self.flow_control_port, start=0):
            RS485_RTU_Client.other_clients[index].append(value)

        self.servers = []


    def __check_overlap(self):
        if self.port_no in self.other_clients[0]:
            return True
        if self.dev in self.other_clients[1]:
            return True
        if self.flow_control_port in self.other_clients[2]:
            return True
        return False
    
    def __configure_fc_port(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.flow_control_port, GPIO.OUT)
        GPIO.output(self.flow_control_port, GPIO.HIGH)
    
    def __configure_connection(self, baudrade = 1920, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS):
        self.serial_port = serial.Serial(port= self.dev,
                                         baudrate=baudrade,
                                         parity=parity,
                                         stopbits=stopbits,
                                         bytesize=bytesize)

    def add_device(self, device_adress: int):
        self.servers.append(RS485_RTU_Client.server(device_adress))
    
    def write_single_holding_register(self, device_adress, register_adress, register_value):
        pass

    class server:
        def __init__(self, client: 'RS485_RTU_Client', adress):
            self.client = client
            self.adress = adress
        def write_single_holding_register(self, register_adress, register_value):
            self.client.write_single_holding_register(self.adress, register_adress, register_value)
            #TODO: dodać return

