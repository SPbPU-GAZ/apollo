from types import SimpleNamespace as Namespace
import json

class Config():

    def __init__(self):
        pass
        #load default here

    def load(self, filename):
        with open(filename, 'r') as f:
            _json = json.load(f)
            for parameter in _json:
                setattr(self, parameter, _json[parameter])

    def save(self, filename):
        _json = json.dumps(self.__dict__, indent=4)
        with open(filename, 'w') as f:
            f.write(_json)
    