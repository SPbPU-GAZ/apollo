import os
from PyQt5.QtGui     import QPainter, QPixmap, QImage
from PyQt5.QtCore    import Qt, QSize
from PyQt5.QtWidgets import QLabel

from application.ApolloInterface import ApolloInterface
from application.ModeDescription import MouseMode, ScaleMode
from application.methods.get_record import get_record
from application.methods.get_cluster import get_cluster
from application.methods.check_type import is_int, is_float

class MapField(QLabel):

    def __init__(self, scenario, parent):
        super(MapField, self).__init__(parent=parent)
        self.scenario = scenario
        
        self.reset() #somewhere here load new conf?
        self.setMouseTracking(True)
        self.add_listeners()


    def reset(self):
        self.cfg = self.scenario.get_cfg()
        self.setStyleSheet("background-color: gray;") 
        self.setFixedSize(QSize(self.cfg.MAP_WIDTH, self.cfg.MAP_HEIGHT))

        self.mouse_xy = [0, 0] #some init val
        self.mouse_mode = MouseMode.NONE
        self.scale_mode = ScaleMode.SCALE_ALL

        self.itf = ApolloInterface()
        self.itf.get_pb_from_text_file(self.cfg.IN_FILE)
        self.itf.print_debug()

        # self.data1 = get_record("modules/map_tool/data/c_bts.csv")
        # self.data2 = get_record("modules/map_tool/data/c_bts2.csv")
        # self.data3 = get_record("modules/map_tool/data/c_stb.csv")
        # self.data4 = get_record("modules/map_tool/data/c_stb2.csv")

        #cX, cY = 563100.0, 6186730.0
        cX, cY = 354232.40, 6657409.53
        dc = 512
        
        self.img_data = get_cluster([[cX - dc, cY + 4 * dc], [cX + dc, cY - dc]], self.scenario)
        
        self.processed_img = []

        for img, bb in self.img_data:
            h, w, _ = img.shape
            bpl = 3 * w
            qimg = QImage(img, w, h, bpl, QImage.Format_RGB888)
            self.processed_img.append([QPixmap(qimg), bb])


    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120

        k = 1.1 ** delta
        x = event.x()
        y = event.y()
        
        if self.scale_mode == ScaleMode.SCALE_ALL:
            gx, gy = self.img_to_global([x, y]) #theoretical global cursor pos (before)
            self.cfg.map_scale *= k
            fx, fy = self.global_to_img([gx, gy]) #future img cursor pos

            dx, dy = x - fx, y - fy #cursor diff
            self.cfg.map_xy[0] += int(dx)
            self.cfg.map_xy[1] += int(dy)

        self.update_cord_info()
        self.update()


    def mousePressEvent(self, event):
        mouse_button = event.button()
        self.mouse_xy = [event.x(), event.y()]

        _dict = {
            Qt.LeftButton: MouseMode.MOVE_ALL
        }

        if mouse_button in _dict:
            self.mouse_mode = _dict[mouse_button]

        #print(self.cfg.mouse_xy, mouse_button)

    
    def mouseReleaseEvent(self, event):
        self.mouse_mode = MouseMode.NONE
    

    def mouseMoveEvent(self, event):
        _x, _y = event.x(), event.y()

        dx, dy = _x - self.mouse_xy[0],  _y - self.mouse_xy[1]

        def move_map():
            self.cfg.map_xy[0] = self.cfg.map_xy[0] + dx
            self.cfg.map_xy[1] = self.cfg.map_xy[1] + dy

        if self.mouse_mode == MouseMode.MOVE_ALL:
            move_map()

        self.mouse_xy = [_x, _y]
        self.update_cord_info()
        self.update()


    def update_cord_info(self):
        _x, _y = self.mouse_xy
        self.scenario['cur_pos_img'].setText(f"{_x},\t{_y}")

        _gx, _gy = self.img_to_global(self.mouse_xy)
        self.scenario['cur_pos_global'].setText(f"{_gx},\t{_gy}")

        _converter = self.scenario.get_converter()
        _lon, _lat = _converter.utm_to_lonlat(_gx, _gy)
        self.scenario['cur_pos_lonlat'].setText(f"{_lon},\t{_lat}")

        _ox, _oy = self.cfg.map_xy
        self.scenario['offset_img'].setText(f"{_ox},\t{_oy}")

        _ogx, _ogy = self.cfg.global_xy
        self.scenario['offset_global'].setText(f"{_ogx},\t{_ogy}")

        _scale = self.cfg.map_scale
        self.scenario['scale'].setText(f"{_scale}")


    def add_listeners(self):
        
        def offset_img_callback():
            text = self.scenario['offset_img'].text()
            try:
                x, y = text.split(',\t')
            except ValueError:
                print("incorrect format")
                return
            
            if not is_int(x) or not is_int(y):
                print("value is not int")
                return
            
            x, y = int(x), int(y)
            self.cfg.map_xy = [x, y]
            self.update_cord_info()
            self.update()

        def offset_global_callback():
            text = self.scenario['offset_global'].text()
            try:
                x, y = text.split(',\t')
            except ValueError:
                print("incorrect format")
                return
            
            if not is_float(x) or not is_float(y):
                print("value is not float")
                return
            
            x, y = float(x), float(y)
            self.cfg.global_xy = [x, y]
            self.update_cord_info()
            self.update()

        def scale_callback():
            scale_txt = self.scenario['scale'].text()
            
            if not is_float(scale_txt):
                print("value is not float")
                return
            
            scale = float(scale_txt)
            self.cfg.map_scale = scale
            self.update_cord_info()
            self.update()



        self.scenario['offset_img'].returnPressed.connect(offset_img_callback)
        self.scenario['offset_global'].returnPressed.connect(offset_global_callback)
        self.scenario['scale'].returnPressed.connect(scale_callback)
        


    def img_to_global(self, xy):
        _x =     (xy[0] - self.cfg.map_xy[0]) / self.cfg.map_scale + self.cfg.global_xy[0] # absolute global
        _y = - ( (xy[1] - self.cfg.map_xy[1]) / self.cfg.map_scale - self.cfg.global_xy[1] )
        return [_x, _y]


    def global_to_img(self, xy):
        _x = ( - self.cfg.global_xy[0] + xy[0]) * self.cfg.map_scale + self.cfg.map_xy[0]
        _y = (   self.cfg.global_xy[1] - xy[1]) * self.cfg.map_scale + self.cfg.map_xy[1]
        return [_x, _y]
    

    def paintEvent(self, event):
        self.drawBackground()
        #self.drawRecorded()
        self.drawMap()


    def drawMap(self):

        def paint_seq(_points, _color):
            painter.setPen(_color)
            for (A, B) in _points:
                Ax, Ay = self.global_to_img([A.x, A.y])
                Bx, By = self.global_to_img([B.x, B.y])

                painter.drawLine(Ax, Ay, Bx, By)

        painter = QPainter(self)

        map = self.itf.map()
        lanes = map.lane

        for lane in lanes:
            points = lane.central_curve.segment[0].line_segment.point #reference trajectory
            points_zip  = list(zip(points[1:], points[:-1]))

            paint_seq(points_zip, Qt.red)

            points = lane.left_boundary.curve.segment[0].line_segment.point #reference trajectory
            points_zip  = list(zip(points[1:], points[:-1]))

            paint_seq(points_zip, Qt.black)

            points = lane.right_boundary.curve.segment[0].line_segment.point #reference trajectory
            points_zip  = list(zip(points[1:], points[:-1]))

            paint_seq(points_zip, Qt.black)


    def drawBackground(self):
        painter = QPainter(self)
        #painter.setPen(Qt.blue)

        for img, bb in self.processed_img:
            Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = bb
            iAx, iAy = self.global_to_img([Ax, Ay])
            iBx, iBy = self.global_to_img([Bx, By])
            iCx, iCy = self.global_to_img([Cx, Cy])
            iDx, iDy = self.global_to_img([Dx, Dy])
            painter.drawPixmap(iAx,   iAy,   iCx - iAx,   iCy - iAy, img)
            painter.setPen(Qt.blue)
            painter.drawLine(iAx, iAy, iCx, iCy)
            painter.setPen(Qt.red)
            painter.drawLine(iBx, iBy, iDx, iDy)
            
        # print(Ax, Ay, Cx, Cy)
        # print(Ax,   Ay,   Cx - Ax,   Cy - Ay)


    def drawRecorded(self):
        painter = QPainter(self)
        painter.setPen(Qt.blue)

        for data in [self.data1, self.data2, self.data3, self.data4]:
            data = data[0::10] # some optimization
            points = list(zip(data["x"], data["y"]))
            for point in points:
                Px, Py = self.global_to_img([point[0], point[1]])
                painter.drawPoint(Px, Py)


            