

class u():
    def __init__(self,board):
        self._board = board

    @property
    def distance(self):
        return 3.0-self._board.port

class mBuild():
    def __init__(self,port=0):
        self._port = port
    
    def connect(self,port=0):
        return mBuild(port)
    
    @property
    def port(self):
        return self._port

    @property
    def ultrasonic(self):
        return Ultrasonic(self)

    def _Ultrasonic(self):
        return u(self)

class Ultrasonic():
    def __init__(self,board):
        self._board = board
        self._ultrasonic = [self._board._Ultrasonic()]

    def distance(self,idx=1):
        return self._ultrasonic[idx-1].distance

boards = {}
def connect(port=0):
    global boards
    key_port = '{0}'.format(port)
    if key_port in boards:
        return boards[key_port]
    boards[key_port]=mBuild(port)
    return boards[key_port]

ultrasonic = connect(0).ultrasonic