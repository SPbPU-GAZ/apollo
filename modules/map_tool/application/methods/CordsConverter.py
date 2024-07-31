import pyproj


class CordsConverter():

    def __init__(self, rtv):
        self.p2 = pyproj.Proj(rtv.gnss_settings, preserve_units=False)
        self.rtv = rtv


    def utm_to_lonlat(self, x, y):
        lonA, latA = self.p2(x, y, inverse=True)
        return [lonA, latA]


    def lonlat_to_utm(self, lon, lat):
        x, y = self.p2(lon, lat)
        return [x, y]
    

    def img_to_global(self, xy):
        _x =     xy[0] / self.rtv.map_scale + self.rtv.global_xy[0] # absolute global
        _y = - ( xy[1] / self.rtv.map_scale - self.rtv.global_xy[1] )
        return [_x, _y]


    def global_to_img(self, xy):
        _x = ( - self.rtv.global_xy[0] + xy[0]) * self.rtv.map_scale
        _y = (   self.rtv.global_xy[1] - xy[1]) * self.rtv.map_scale
        return [_x, _y]
    

    def img_to_gl(self, xy):
        _x =    2 * xy[0] / self.rtv.MAP_WIDTH - 1
        _y = - (2 * xy[1] / self.rtv.MAP_HEIGHT - 1) 
        return [_x, _y]
    

    def global_to_gl(self, xy):
        return self.img_to_gl(self.global_to_img(xy))
