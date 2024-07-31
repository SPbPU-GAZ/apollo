import os
from application.RuntimeVars import RuntimeVars
from application.methods.TileService import Services
from application.methods.CordsConverter import CordsConverter

DEFAULT_CFG_FILE = os.path.join("modules/map_tool/data/confs", "cfg_default.json")


class Scenario():

    def __init__(self):
        self._rtv = None
        self._converter = None
        self._widgets = {}
        self.load_default_conf()


    def load_conf(self, rtv_file):
        self._rtv = RuntimeVars()
        self._rtv.load(rtv_file)
        self._converter = CordsConverter(self._rtv)
        self._service = Services.SatelliteImage


    def load_default_conf(self):
        self.load_conf(DEFAULT_CFG_FILE)


    def save_conf(self, rtv_file):
        self._rtv.save(rtv_file)

    def get_rtv(self):
        return self._rtv
    

    def get_converter(self):
        return self._converter
    

    def get_map_api_service(self):
        return self._service
    

    def __setitem__(self, name, widget):
        self._widgets[name] = widget


    def __getitem__(self, name):
        return self._widgets[name]