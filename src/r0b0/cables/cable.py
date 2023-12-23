from abc import abstractmethod

class Cable(object):
    def __init__(self,):
        self.input_event = ''

    @abstractmethod
    def __call__(self, data: dict):
        return {}

class Motion2MotorCable(Cable):
    def __init__(self,):
        super().__init__()

    def __call__(self,):
        return {
            'event':'position'
        }

class Key2MouseCable(Cable):
    def __init__(self,):
        self.input_event = 'keydown'
    
    def __call__(self, data):
        key2pos_dict = {
            'q':[100,100],
            'w':[500,100],
            'e':[900,100],
            'a':[100,400],
            's':[500,400],
            'd':[900,400],
            'z':[100,700],
            'x':[500,700],
            'c':[900,700],
        }
        # key2pos_dict.setdefault([500,400])
        [x,y] = key2pos_dict.get(data['unicode'],[500,400])
        print(x,y)
        return {
            'event':'mouse_place',
            'x':x,
            'y':y,
        }