from enum import Enum

#https://contextily.readthedocs.io/en/stable/intro_guide.html
class Services(Enum):

    SatelliteImage = ('satellite', 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}')
    OSMImage = ('osm', 'https://tile.openstreetmap.org/{z}/{x}/{y}.png')

    def __init__(self, name, url):
        self._name = name
        self._url = url


    def name(self):
        return self._name


    def url(self):
        return self._url