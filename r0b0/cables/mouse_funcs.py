def key2mouse_move(data=None):
    if data is None: return {'event':'keydown'}
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
    return {
        'event':'mouse_place',
        'x':x,
        'y':y,
    }