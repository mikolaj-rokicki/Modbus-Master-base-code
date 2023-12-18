import RPi.GPIO as GPIO
import serial
from time import sleep
import logging
from typing import Union

class Modbus_Exception(Exception):
    def __init__(self, value):
        self.value = value

class Initialization_Exception(Modbus_Exception):
    def __init__(self, value):
        self.value = value
class Transmission_Exception(Modbus_Exception):
    def __init__(self, value):
        self.value = value
class No_Connection_Exception(Transmission_Exception):
    def __init__(self, slave):
        self.value = f'No connection from slave {slave}'
        logging.log(logging.WARNING, self.value) 
class Connection_Interrupted_Exception(Transmission_Exception):
    def __init__(self, slave, value):
        self.value = f'Slave {slave}: {value}'
        logging.log(logging.WARNING, self.value)

class RS485_RTU_Master:
    PORTS_DEFAULTS = ((None, 10), ('/dev/TTYAMA3', 27), ('/dev/TTYAMA4', 7), ('/dev/TTYAMA0', 21))
    other_clients = [[],[],[]] #ports, devs, f_c ports

    # START of Initial configuration
    def __init__(self, port_no: int = None, dev = None, flow_control_port: int = None, **kwargs):
        if port_no is None:
            logging.log(logging.WARNING, 'RS485_RTU_Master initialisation passed!')
            return
        self.__overlap_checking = True
        if type(port_no) is not int:
            raise Initialization_Exception('port_no has to be an integer')
        # Assign default values
        if dev is None or flow_control_port is None:
            if len(RS485_RTU_Master.PORTS_DEFAULTS)-1 < port_no:
                raise Initialization_Exception('port_no not in range of supported defaults')
            if dev == None:
                dev = RS485_RTU_Master.PORTS_DEFAULTS[port_no][0]
                if dev == None:
                    raise Initialization_Exception('Not avialable default device name for this port')
            if flow_control_port == None:
                flow_control_port = RS485_RTU_Master.PORTS_DEFAULTS[port_no][1]
                if flow_control_port == None:
                    raise Initialization_Exception('Not avialable default flow control port for this port')
        if type(dev) is not str:
            raise Initialization_Exception('argument dev has to be string')
        if type(flow_control_port) is not int:
            raise Initialization_Exception('argument flow_control_port has to be a string')

        self.port_no = port_no
        self.dev = dev
        self.flow_control_port = flow_control_port

        if self.__overlap_checking:
            if self.__check_overlap():
                raise Initialization_Exception('Found overlaping ports/device names to turn off this exception use method configure_overlap_checking(False)')
        self.__configure_fc_port()
        self.__configure_connection()
        for index, value in enumerate(self.port_no, self.dev, self.flow_control_port, start=0):
            RS485_RTU_Master.other_clients[index].append(value)

        self.servers = []

    def convigure_overlap_checking(self, value: bool):
        if type(value) is not bool:
            raise Modbus_Exception('value of argument "value" has to be an bool')
        self.__overlap_checking = value

    def __check_overlap(self):
        if self.port_no in self.other_clients[0]:
            return True
        if self.port_no in self.other_clients[2]:
            return True
        if self.dev in self.other_clients[1]:
            return True
        if self.flow_control_port in self.other_clients[0]:
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
        
        for i in range(1, 6):
            try:
                received_data = b''
                GPIO.output(self.flow_control_port, GPIO.LOW)
                self.serial_port.write(data)
                sleep(0.006) #TODO: Calculate sleep time
                GPIO.output(self.flow_control_port, GPIO.HIGH)
                sleep(0.1) #TODO: Calculate sleep time
                data_left = self.serial_port.inWaiting()
                if data_left == 0:
                    raise No_Connection_Exception(data[0])
                logging.log(logging.DEBUG, f'reading {str(data_left)} bytes of return message')
                received_data += self.serial_port.read(data_left)
                logging.log(logging.INFO, f'RECEIVED DATA: {received_data}')
                success = True
            except Transmission_Exception:
                success = False
            except:
                return #TODO: add return type
            if success is True:
                break            
            

        return received_data
        #TODO: add exceptions
    # END of transfer functions

    def __check_if_response_is_propper(self, data, response):
        if len(response<6):
            raise Connection_Interrupted_Exception(data[0], 'Response length too short')
        if data[0]!=response[0]:
            raise Connection_Interrupted_Exception(data[0], 'Response adress doesn\'t match with desired')
        if data[1]!=response[1] and data[1]+128 != response[1]:
            raise Connection_Interrupted_Exception(data[0], 'Function codes from connection and response doesn\'t match')
        #TODO: check if crc matches
        response_data = response[:-2]
        crc = response[-2:]
        expected_crc = self.__calculate_CRC(response_data)
        if crc != expected_crc:
            raise Connection_Interrupted_Exception(data[0], f'CRC doesn\'t match, received CRC: {crc}, expected CRC: {expected_crc}') 

    def add_slave(self, slave_adress: int):
        self.servers.append(RS485_RTU_Master.Slave(slave_adress))
    
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

    def write_single_holding_register(self, slave_adress: Union[int, bytes], register_adress: Union[int, bytes], register_value: int):
        # checking if slave adress is propper
        if type(slave_adress) is int:
            if slave_adress < 1 or slave_adress > 247:
                raise Modbus_Exception(f'value {slave_adress} is unsupported for slave_adress, it has to be an integer between 1 and 247!')
            slave_adress = slave_adress.to_bytes(1)
        elif type(slave_adress) is bytes:
            if len(slave_adress) != 1:
                raise Modbus_Exception(f'length of argument slave_adress is {len(slave_adress)}, it has to be 1 byte')
            if slave_adress[0] < 1 or slave_adress[0] > 247:
                raise Modbus_Exception(f'value {slave_adress} is unsupported for slave_adress, it has to be an integer between 00 and F7')
        else:
            raise Modbus_Exception(f'Type {type(slave_adress)} is unsupported for slave_adress, supported types are: int and bytes!')
        # assign function code
        function_code = b'\x06'
        # checking if register adress is propper
        if type(register_adress) is int:
            if register_adress < 0 or register_adress > 65535:
                raise Modbus_Exception(f'value {register_adress} is unsupported for register_adress, it has to be an integer between 0 and 65535')
            register_adress = register_adress.to_bytes(2)
        elif type(register_adress) is bytes:
            if len(register_adress) != 2:
                raise Modbus_Exception(f'length of argument register_adress is {len(register_adress)}, it has to be 2 bytes')
        else:
            raise Modbus_Exception(f'Type {type(register_adress)} is unsupported for register_adress, supported types are: int and bytes!')

        #TODO: register value exceptions
        register_value = register_value.to_bytes(2)
        data = slave_adress+function_code+register_adress+register_value
        data = data+self.__calculate_CRC(data)
        logging.log(logging.INFO, f'Prepared message to send: {data.hex()}')
        #self.__send_data(data)
        logging.log(logging.WARNING, 'Data sending is turned off')
        #TODO: enable send data


    class Slave:
        def __init__(self, master: 'RS485_RTU_Master', adress: int):
            self.client = master
            self.adress = adress
        def write_single_holding_register(self, register_adress, register_value):
            self.client.write_single_holding_register(self.adress, register_adress, register_value)
            #TODO: dodać return




