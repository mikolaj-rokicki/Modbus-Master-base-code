import RPi.GPIO as GPIO
import serial
from time import sleep
import logging

class RS485_RTU_Master:
    PORTS_DEFAULTS = ((None, 10), ('/dev/TTYAMA3', 27), ('/dev/TTYAMA4', 7), ('/dev/TTYAMA0', 21))
    other_clients = [[],[],[]] #ports, devs, f_c ports

    # START of Initial configuration
    def __init__(self, port_no: int = None, dev = None, flow_control_port: int = None, **kwargs):
        if port_no is None:
            return
        # Assign default values
        if dev == None:
            if len(RS485_RTU_Master.PORTS_DEFAULTS)-1 < port_no:
                #TODO: add exception
                pass
            dev = RS485_RTU_Master.PORTS_DEFAULTS[port_no][0]
            if dev == None:
                #TODO: add exception
                pass
        if flow_control_port == None:
            if len(RS485_RTU_Master.PORTS_DEFAULTS)-1 < port_no:
                #TODO: add exception
                pass
            flow_control_port = RS485_RTU_Master.PORTS_DEFAULTS[port_no][1]
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
            RS485_RTU_Master.other_clients[index].append(value)

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
    # END of initial configuration

    # START Transfer functions
    def __send_data(self, data):
        received_data = b''
        GPIO.output(self.flow_control_port, GPIO.LOW)
        self.serial_port.write(data)
        sleep(0.006) #TODO: Calculate sleep time
        GPIO.output(self.flow_control_port, GPIO.HIGH)
        sleep(0.1) #TODO: Calculate sleep time
        data_left = self.serial_port.inWaiting()
        logging.log(logging.DEBUG, f'reading {str(data_left)} bytes of return message')
        received_data += self.serial_port.read(data_left)
        logging.log(logging.INFO, f'RECEIVED DATA: {received_data}')
        return received_data
        #TODO: add exceptions
    # END of transfer functions

    def add_slave(self, slave_adress: int):
        self.servers.append(RS485_RTU_Master.slave(slave_adress))
    
    def __calculate_CRC(self, data):
        #TODO: fast conversion
        crc = 0xFFFF
        for n in range(len(data)):
            crc ^= data[n]
            for i in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return(crc.to_bytes(2, 'little'))

    def write_single_holding_register(self, slave_adress: int, register_adress: int, register_value: int):
        #TODO: check if valid adress
        #TODO: add bytes as assepted argument type
        slave_adress = slave_adress.to_bytes(1)
        function_code = b'\x06'
        register_adress = register_adress.to_bytes(2)
        register_value = register_value.to_bytes(2)
        data = slave_adress+function_code+register_adress+register_value
        data = data+self.__calculate_CRC(data)
        logging.log(logging.INFO, f'Prepared message to send: {data.hex()}')
        #self.__send_data(data)
        #TODO: enable send data
        pass


    class slave:
        def __init__(self, master: 'RS485_RTU_Master', adress: int):
            self.client = master
            self.adress = adress
        def write_single_holding_register(self, register_adress, register_value):
            self.client.write_single_holding_register(self.adress, register_adress, register_value)
            #TODO: dodać return

