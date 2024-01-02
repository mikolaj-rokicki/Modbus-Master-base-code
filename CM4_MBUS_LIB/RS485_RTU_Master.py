import RPi.GPIO as GPIO
import serial
from time import sleep
import logging
from typing import Union, Literal
import math

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
    PORTS_DEFAULTS = ((None, 10), ('/dev/ttyAMA3', 27), ('/dev/ttyAMA4', 7), ('/dev/ttyAMA0', 21))
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
        RS485_RTU_Master.other_clients[0].append(port_no)
        RS485_RTU_Master.other_clients[1].append(dev)
        RS485_RTU_Master.other_clients[2].append(flow_control_port)

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
    
    def __configure_connection(self, baudrade = 19200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS):
        self.baudrate = baudrade
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
                sending_time = self.__calculate_time(len(data))
                self.serial_port.write(data)
                sleep(sending_time)
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

        return received_data[-2:-2]
        #TODO: add exceptions

    def __calculate_time(self, bytes_no):
        return (bytes_no+4)*9/self.baudrate
        #TODO: make sure its good formula
    # END of transfer functions

    def __check_if_response_is_propper(self, data, response):
        if len(response<6):
            raise Connection_Interrupted_Exception(data[0], 'Response length too short')
        if data[0]!=response[0]:
            raise Connection_Interrupted_Exception(data[0], 'Response adress doesn\'t match with desired')
        if data[1]!=response[1] and data[1]+128 != response[1]:
            raise Connection_Interrupted_Exception(data[0], 'Function codes from connection and response doesn\'t match')
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

    def read_discrete_inputs(self, slave_adress: int | bytes, starting_adress: int | bytes, inputs_qty: int | bytes) -> list[bool]:
        return self.__read_discrete(slave_adress, starting_adress, inputs_qty, 'DI')
    
    def read_coils(self, slave_adress: int | bytes, starting_adress: int | bytes, inputs_qty: int | bytes) -> list[bool]:
        return self.__read_discrete(slave_adress, starting_adress, inputs_qty, 'Coils')
    
    def __read_discrete(self, slave_adress: int | bytes, starting_adress: int | bytes, inputs_qty: int | bytes, fc: Literal['DI', 'Coils']) -> list[bool]:
        slave_adress = self.__check_and_convert_slave_adress(slave_adress)
        starting_adress = self.__check_if_int_or_byte_and_convert_in_bounds(starting_adress, 2, 0, int(0xFFFF))
        inputs_qty = self.__check_if_int_or_byte_and_convert_in_bounds(inputs_qty, 2, 1, 2000)
        if(fc) == 'DI':
            function_code = b'\x02'
        elif(fc) == 'Coils':
            function_code = b'\x01'
        response = self.__send_data(slave_adress+function_code+starting_adress+inputs_qty)
        coils = int.from_bytes(inputs_qty)
        n = math.ceil(coils/8)
        if len(response)!=(n+1):
            raise Transmission_Exception(f'Received {len(response)} bytes of data, expected {n+1} bytes')
        return self.__convert_coil_state(response[-1:])
    
    def write_single_coil(self, slave_adress: int | bytes, output_adress: int | bytes, value: bool):
        slave_adress = self.__check_and_convert_slave_adress(slave_adress)
        starting_adress = self.__check_if_int_or_byte_and_convert_in_bounds(starting_adress, 2, 0, int(0xFFFF))
        if bool(value) is True:
            value = b'\xFF\x00'
        else:
            value = b'\x00\x00'
        function_code = b'\x05'
        self.__send_data(slave_adress+function_code+starting_adress+value)

    def __convert_coil_state(data: bytearray, coil_qty: int) -> list[bool]:
        bool_list = [bool((1 << i) & byte) for byte in data for i in range(0,8)]
        return bool_list[:coil_qty]



    def __check_if_int_or_byte_and_convert_in_bounds(self, value, bytes_qty = None, lower_bound: int = 0, upper_bound: int | float = float('inf'), byteorder='big') -> bytes:
        if type(value) is int:
            if(value<lower_bound or value>upper_bound):
                raise Modbus_Exception(f'value {value} is outside of bounds, it has to be between {lower_bound} and {upper_bound}')
            return value.to_bytes(bytes_qty)
        elif type(value) is bytes:
            if bytes_qty is not None:
                if(len(value) != bytes_qty):
                    raise Modbus_Exception(f'length of value {value} is {len(value)}, it has to be {bytes_qty} bytes')
            if (int.from_bytes(value, byteorder)<lower_bound or int.from_bytes(value, byteorder>upper_bound)):
                raise Modbus_Exception(f'value {int.from_bytes(value, byteorder)} is outside of bounds, it has to be between {lower_bound} and {upper_bound}')
            return value
        else:
            raise Modbus_Exception(f'type {type(value)} is supposed to be int or bytes') 

    def write_single_holding_register(self, slave_adress: Union[int, bytes], register_adress: Union[int, bytes], register_value: int):
        # slave adress
        slave_adress = self.__check_and_convert_slave_adress()
        # assign function code
        function_code = b'\x06'
        # checking if register adress is propper
        register_adress = self.__check_and_convert_register_adress(register_adress)

        #TODO: register value exceptions
        register_value = register_value.to_bytes(2)
        data = slave_adress+function_code+register_adress+register_value
        data = data+self.__calculate_CRC(data)
        logging.log(logging.INFO, f'Prepared message to send: {data.hex()}')
        #self.__send_data(data)
        logging.log(logging.WARNING, 'Data sending is turned off')
        #TODO: enable send data

    def write_multiple_holding_registers(self, slave_adress: Union[int, bytes], starting_register_adress: Union[int, bytes], registers_qty: Union[int, bytes], values: list[Union[int, bytes]]):
        self.__check_and_convert_slave_adress()
        function_code = b'\x10'
        starting_register_adress = self.__check_and_convert_register_adress(starting_register_adress)

        # registers_qty validation
        if type(registers_qty) is int:
            if registers_qty < 1 or registers_qty > 123:
                raise Modbus_Exception(f'value {registers_qty} is unsupported for registers_qty, it has to be an integer between 1 and 123!')
            registers_qty = registers_qty.to_bytes(2)
        elif type(registers_qty) is bytes:
            if len(registers_qty) != 2:
                raise Modbus_Exception(f'length of argument registers_qty is {len(registers_qty)}, it has to be 2 bytes')
            if registers_qty[1] < 1 or registers_qty[1] > 247:
                raise Modbus_Exception(f'value {registers_qty} is unsupported for registers_qty, it has to be an integer between 0000 and 007B')
        else:
            raise Modbus_Exception(f'Type {type(registers_qty)} is unsupported for registers_qty, supported types are: int and bytes!')
        
        # byte count
        byte_count = (registers_qty[1]*2).to_bytes(1)

        # values validation
        values_bytes = b''
        if type(values)!=list:
            raise Modbus_Exception(f'Type {type(values)} is unsupported for values, supported types is list of int and bytes!')
        if len(values)!=registers_qty[1]:
            raise Modbus_Exception(f'length of values: {len(values)} doesn\'t match registers_qty: {len(registers_qty)}')
        for index, value in enumerate(values, start=0):
            if type(value) is int:
                if value < 0 or value > 65535:
                    raise Modbus_Exception(f'values[{index}]: {value} is unsupported for values, it has to be an integer between 0 and 65535!')
                value = value.to_bytes(2)
                values_bytes += value
            elif type(value) is bytes:
                if len(value) != 2:
                    raise Modbus_Exception(f'length of argument values[{index}] is {len(value)}, it has to be 2 bytes')
                values_bytes += value
            else:
                raise Modbus_Exception(f'Type {type(value)} is unsupported for values, supported types are: int and bytes!')
            
        data = slave_adress+function_code+starting_register_adress+registers_qty+byte_count+values_bytes
        data = data+self.__calculate_CRC(data)
        logging.log(logging.INFO, f'Prepared message to send: {data.hex()}')
        #self.__send_data(data)
        logging.log(logging.WARNING, 'Data sending is turned off')
        #TODO: enable send data

    def __check_and_convert_slave_adress(self, slave_adress) -> bytes:
        return self.__check_if_int_or_byte_and_convert_in_bounds(slave_adress, 1, 0, 247)

    def __check_and_convert_register_adress(self, register_adress):
        if type(register_adress) is int:
            if register_adress < 0 or register_adress > 65535:
                raise Modbus_Exception(f'value {register_adress} is unsupported for register_adress, it has to be an integer between 0 and 65535')
            register_adress = register_adress.to_bytes(2)
        elif type(register_adress) is bytes:
            if len(register_adress) != 2:
                raise Modbus_Exception(f'length of argument register_adress is {len(register_adress)}, it has to be 2 bytes')
        else:
            raise Modbus_Exception(f'Type {type(register_adress)} is unsupported for register_adress, supported types are: int and bytes!')
        return register_adress


    class Slave:
        def __init__(self, master: 'RS485_RTU_Master', adress: int):
            self.client = master
            self.adress = adress
        def write_single_holding_register(self, register_adress, register_value):
            self.client.write_single_holding_register(self.adress, register_adress, register_value)
            #TODO: dodać return

        #TODO: add remaining functions
