from dds import Listener, DomainParticipant

# Data available listener
class DataAvailableListener(Listener):
    def __init__(self):
        Listener.__init__(self)

    def on_data_available(self, entity):
        print('on_data_available called')
        l = entity.read(10)
        for (sd, si) in l:
            sd.print_vars()

dp = DomainParticipant()
topic = dp.create_topic('foo_topic')
sub = dp.create_subscriber()
sub.create_datareader(topic,listener=DataAvailableListener())

while True:
    pass