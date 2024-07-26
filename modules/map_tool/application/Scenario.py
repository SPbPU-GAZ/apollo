import os
from application.Config import Config
from application.methods.TileService import Services
from application.methods.CordsConverter import CordsConverter

DEFAULT_CFG_FILE = os.path.join("modules/map_tool/data", "cfg.json")


class Scenario():

    def __init__(self, load_default=False):
        self._cfg = None
        self._converter = None
        self._widgets = {}
        
        if load_default:
            self.load_default()


    def load_conf(self, cfg_file):
        self._cfg = Config()
        self._cfg.load(cfg_file)
        self._converter = CordsConverter(self._cfg.gnss_settings)
        self._service = Services.SatelliteImage


    def load_default(self):
        self.load_conf(DEFAULT_CFG_FILE)


    def get_cfg(self):
        return self._cfg
    

    def get_converter(self):
        return self._converter
    

    def get_map_api_service(self):
        return self._service
    

    def __setitem__(self, name, widget):
        self._widgets[name] = widget


    def __getitem__(self, name):
        return self._widgets[name]