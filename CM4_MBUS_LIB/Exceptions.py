class Modbus_Exception(Exception):
    def __init__(self, value):
        self.value = value

class Initialization_Exception(Modbus_Exception):
    def __init__(self, value):
        self.value = value