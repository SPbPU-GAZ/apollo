import pyproj


class CordsConverter():

    def __init__(self, gnss_settings):
        self.p2 = pyproj.Proj(gnss_settings, preserve_units=False)

    def utm_to_lonlat(self, x, y):
        lonA, latA = self.p2(x, y, inverse=True)
        return [lonA, latA]

    def lonlat_to_utm(self, lon, lat):
        x, y = self.p2(lon, lat)
        return [x, y]
