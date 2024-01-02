from CM4_MBUS_LIB import RS485_RTU_Master



#master = RS485_RTU_Master.RS485_RTU_Master(1)
#master.write_single_holding_register(1, 4, int.from_bytes(b'\xAB\xCD'))
#master.write_single_holding_register()


byte_array = b'\xAC\xDB\xFB\x0D'
count = 28

#bool_list = [bool((1 << i) & byte) for i in range(0,8) for byte in range(0, 2)]

bool_list = [bool((1 << i) & byte) for byte in byte_array for i in range(0,8)]
bool_list = bool_list[:count]


print([int(x) for x in bool_list])


#bool_list = [b == 1 for byte in byte_array for b in reversed([bool(byte & (1 << i)) for i in range(8)])]
