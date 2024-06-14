import can


class CAN:
    
    __instance = {}

    @staticmethod
    def getInstance(port):
        if CAN.__instance.get(port,None) == None:
            CAN(port)
        return CAN.__instance[port]

    def __init__(self, port):
        if CAN.__instance.get(port,None) != None:
            raise Exception("CAN exists already!")
        else:
            CAN.__instance[port] = self
            self.bus = can.Bus(port, bustype='socketcan')
    
    def message(self, arb_id, extended, dat, remote = False):
        msg = can.Message(arbitration_id=arb_id, is_extended_id=extended, data=dat, is_remote_frame = remote)
        return msg
    
    def send(self, msg):
        self.bus.send(msg)

    def recv(self,sec=None):
        if sec != None:
            return self.bus.recv(sec)
        else:
            return self.bus.recv()

