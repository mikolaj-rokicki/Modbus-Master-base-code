

class RequestRHR:
    functionCode = bytearray([0x03])
    def __init__(self, slaveID, startAdress, length):
        self.PDU = self.functionCode+startAdress.to_bytes(2)+length.to_bytes(2)
        self.slaveID = slaveID.to_bytes(2)


class Message:
    def __init__(self, deviceAdress, PDU):
        self.message = deviceAdress+PDU+self.createCRC(deviceAdress+PDU)

    def createCRC(self, pMessage):
        crc = 0xFFFF
        for n in range(len(pMessage)):
            crc ^= pMessage[n]
            for i in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1

        return(crc.to_bytes(2))

        
#slave ID = 1
#start adress = 0
#length = 1
#PDU = 0300000001
#CRC = 119F

#message = 00 01 03 00 00 00 01 11 9F

#querry = RequestRHR(1, 0, 1)
#message = Message(querry.slaveID, querry.PDU)
#print(message.message)

function_code = b'\x06'
device_adress = b'\x12'
data = function_code+device_adress
print(data)

function_code.
