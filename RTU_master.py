from CM4_MBUS_LIB import RS485_RTU_Master



master = RS485_RTU_Master.RS485_RTU_Master()
master.write_single_holding_register(11, 4, int.from_bytes(b'\xAB\xCD'))

