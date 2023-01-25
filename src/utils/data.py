import pickle

def dump_pickle(func):
    return lambda s, event, data, **kwargs: func(
        s, event, data=pickle.dumps(data), **kwargs)

def load_pickle(func):
    return lambda s,data,**kwargs: func(
        s, data=pickle.loads(data),**kwargs)
