import os
import numpy as np
from PIL import Image
from PyQt5.QtGui     import QPainter, QPixmap, QImage, QOpenGLVersionProfile
from PyQt5.QtCore    import Qt, QSize
from PyQt5.QtWidgets import QLabel, QOpenGLWidget, QFileDialog, QAction

from application.ApolloInterface import ApolloInterface
from application.ModeDescription import MouseMode, ScaleMode, FIXMode
from application.methods.get_record import get_record, get_strange_record
from application.methods.get_cluster import get_cluster
from application.methods.check_type import is_int, is_float

class MapField(QOpenGLWidget):

    def __init__(self, scenario, parent):
        super(MapField, self).__init__(parent=parent)
        self.scenario = scenario
        self.reset()

        self.setMouseTracking(True)
        self.add_listeners()


    def reset(self):

        if hasattr(self, 'binded_data'):
            self.clear_loaded_area()

        if hasattr(self, 'record_data') and hasattr(self, "local_record_actions"):
            self.clear_record_files()

        self.rtv = self.scenario.get_rtv()
        self.converter = self.scenario.get_converter()
        self.setFixedSize(QSize(self.rtv.MAP_WIDTH, self.rtv.MAP_HEIGHT))
        
        self.mouse_xy = [0, 0]
        self.selected_area = None

        self.binded_data = {}
        self.record_data = []
        self.local_record_actions = []
        #353544.0,	6657785.0
        #562223.0,  6186964.0

        self.mouse_mode = MouseMode.NONE
        self.fix_mode = FIXMode.NONE
        self.scale_mode = ScaleMode.SCALE_ALL

        if hasattr(self, 'gl'):
            self.bind_area(self.rtv.background_roi)
        
        for recordfile, type in self.rtv.records:
            self.add_record_file(recordfile, type, init=True)

        self.new_map()
        if not self.rtv.MAP_FILE == "":    
            self.load_map(self.rtv.MAP_FILE)

        self.scenario['menu_draw_map'].setChecked(self.rtv.draw_map)
        self.scenario['menu_draw_records'].setChecked(self.rtv.draw_records)
        self.scenario['menu_draw_background'].setChecked(self.rtv.draw_background)
        #car enable here too


    def load_map(self, filepath):
        self.new_map()
        self.itf.get_pb_from_text_file(filepath)
        self.itf.print_debug()

    
    def save_map(self, filepath):
        self.itf.write_pb_to_text_file(filepath)


    def new_map(self):
        self.itf = ApolloInterface()


    def load_selected_area(self):
        x1, y1 = self.selected_area[0]
        x2, y2 = self.selected_area[1]
        Ax, Ay = min(x1, x2), max(y1, y2)
        Cx, Cy = max(x1, x2), min(y1, y2)
        roi = [[Ax, Ay], [Cx, Cy]]

        self.rtv.background_roi.append(roi) #for cfg loading
        self.bind_area([roi])


    def clear_loaded_area(self):
        self.rtv.background_roi = []
        id_array_to_del = [self.binded_data[key][0] for key in self.binded_data]

        if len(id_array_to_del) == 0:
            return

        self.gl.glDeleteTextures(len(id_array_to_del), id_array_to_del)
        self.gl.glFinish() # as driver does not immidiately delete textures
        self.binded_data = {}
        

    def bind_area(self, rois):
        for A, C in rois:
            Ax, Ay = A
            Cx, Cy = C
            tiles = get_cluster([[Ax, Ay], [Cx, Cy]], self.scenario)
            for name, img, bb in tiles:
                if not name in self.binded_data:
                    id = self.bindTexture(img)
                    self.binded_data[name] = [id, bb]
    

    def add_record_file(self, filename, type, init=False):
        
        if not type in ['apollo', 'gkv']:
            print('incorrect record format')
            return

        if (filename in (fn for fn, _ in self.rtv.records)) and not init:
            return

        if type == 'apollo':
            _raw_data = get_record(filename)

        if type == 'gkv':
            _raw_data = get_strange_record(filename, self.converter)
        
        _data_reduced = list(zip(_raw_data["x"], _raw_data["y"]))[0::self.rtv.RECORD_DEL]
        
        if not init:
            self.rtv.records.append([filename, type])
        
        self.record_data.append(_data_reduced)

        _action = QAction(filename, self)
        self.scenario['menu_submenu_records'].addAction(_action)
        self.local_record_actions.append(_action)


    def clear_record_files(self):
        for action in self.local_record_actions:
            action.deleteLater()
        self.rtv.records = []
        self.record_data = []
        self.local_record_actions = []


    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120
        k = 1.1 ** delta
        x, y = event.x(), event.y()

        if self.scale_mode == ScaleMode.SCALE_ALL:
            gx, gy = self.converter.img_to_global([x, y]) #theoretical global cursor pos (before)
            self.rtv.map_scale *= k
            fx, fy = self.converter.img_to_global([x, y]) #theoretical global cursor pos (after)

            dx, dy = gx - fx, gy - fy #global cord diff
            self.rtv.global_xy[0] += dx
            self.rtv.global_xy[1] += dy

        self.update_cord_info()
        self.update()


    def mousePressEvent(self, event):
        mouse_button = event.button()

        if mouse_button == Qt.LeftButton:
            if self.mouse_mode == MouseMode.SELECT_AREA:
                self.selected_area = [self.converter.img_to_global(self.mouse_xy), None]
                return    
            
            self.mouse_mode = MouseMode.MOVE_ALL
            return

        if mouse_button == Qt.MiddleButton:
            self.fix_mode = FIXMode.FIX_SHOWING_POS if (self.fix_mode == FIXMode.NONE) else FIXMode.NONE
            return


    def mouseReleaseEvent(self, event):
        if self.mouse_mode == MouseMode.SELECT_AREA:
            self.selected_area[1] = self.converter.img_to_global(self.mouse_xy)
            self.load_selected_area()
            self.update()
            self.selected_area = None

        self.mouse_mode = MouseMode.NONE


    def mouseMoveEvent(self, event):
        _x, _y = event.x(), event.y()

        if self.mouse_mode == MouseMode.MOVE_ALL:
            gx, gy = self.converter.img_to_global(self.mouse_xy) #theoretical global cursor pos (before)    
            fx, fy = self.converter.img_to_global([_x, _y])

            dx, dy = gx - fx, gy - fy
    
            self.rtv.global_xy[0] += dx
            self.rtv.global_xy[1] += dy

        self.mouse_xy = [_x, _y]

        self.update_cord_info()
        self.update()


    def update_cord_info(self):
        if self.fix_mode == FIXMode.FIX_SHOWING_POS:
            return
        
        _x, _y = self.mouse_xy
        self.scenario['cur_pos_img'].setText(f"{_x},\t{_y}")

        _gx, _gy = self.converter.img_to_global(self.mouse_xy)
        self.scenario['cur_pos_global'].setText(f"{_gx},\t{_gy}")

        _lon, _lat = self.converter.utm_to_lonlat(_gx, _gy)
        self.scenario['cur_pos_lonlat'].setText(f"{_lon},\t{_lat}")

        _ox, _oy = self.rtv.background_offset
        self.scenario['background_offset'].setText(f"{_ox},\t{_oy}")

        _ogx, _ogy = self.rtv.global_xy
        self.scenario['offset_global'].setText(f"{_ogx},\t{_ogy}")

        _scale = self.rtv.map_scale
        self.scenario['scale'].setText(f"{_scale}")


    def add_listeners(self):

        def background_offset_callback():
            text = self.scenario['background_offset'].text()
            try:
                x, y = text.split(',\t')
            except ValueError:
                print("incorrect format")
                return

            if not is_float(x) or not is_float(y):
                print("value is not float")
                return

            x, y = float(x), float(y)
            self.rtv.background_offset = [x, y]
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
            self.rtv.global_xy = [x, y]
            self.update_cord_info()
            self.update()

        def scale_callback():
            scale_txt = self.scenario['scale'].text()

            if not is_float(scale_txt):
                print("value is not float")
                return

            scale = float(scale_txt)
            self.rtv.map_scale = scale
            self.update_cord_info()
            self.update()

        self.scenario['background_offset'].returnPressed.connect(background_offset_callback)
        self.scenario['offset_global'].returnPressed.connect(offset_global_callback)
        self.scenario['scale'].returnPressed.connect(scale_callback)


        def draw_records_callback(state):
            self.rtv.draw_records = state
            self.update()

        def draw_map_callback(state):
            self.rtv.draw_map = state
            self.update()

        def draw_background_callback(state):
            self.rtv.draw_background = state
            self.update()

        self.scenario['menu_draw_records'].triggered.connect(draw_records_callback)
        self.scenario['menu_draw_map'].triggered.connect(draw_map_callback)
        self.scenario['menu_draw_background'].triggered.connect(draw_background_callback)


        def add_region_callback():
            self.mouse_mode = MouseMode.SELECT_AREA

        def clear_region_callback():
            self.clear_loaded_area()
            self.update()

        self.scenario['menu_background_add'].triggered.connect(add_region_callback)
        self.scenario['menu_background_clear'].triggered.connect(clear_region_callback)


        def open_apollo_records_callback():
            filenames, _ = QFileDialog.getOpenFileNames(self, 'Open File', self.rtv.DATA_DIR, "Record Apollo File (*.csv)")

            for filename in filenames:
                self.add_record_file(filename, 'apollo')
                self.update()

        def open_gkv_records_callback():
            filenames, _ = QFileDialog.getOpenFileNames(self, 'Open File', self.rtv.DATA_DIR, "Record GKV File (*.csv)")

            for filename in filenames:
                self.add_record_file(filename, 'gkv')
                self.update()

        def clear_records_callback():
            self.clear_record_files()
            self.update()

        self.scenario['menu_records_add_apollo'].triggered.connect(open_apollo_records_callback)
        self.scenario['menu_records_add_gkv'].triggered.connect(open_gkv_records_callback)
        self.scenario['menu_records_clear'].triggered.connect(clear_records_callback)

        def load_map_callback():
            filename, _ = QFileDialog.getOpenFileName(self, 'Open File', self.rtv.DATA_DIR, "Map file (*.txt)")
            if filename:
                self.rtv.MAP_FILE = filename
                self.load_map(self.rtv.MAP_FILE)
                self.update()

        def save_map_callback():
            filename, _ = QFileDialog.getSaveFileName(self, 'Open File', self.rtv.DATA_DIR, "Map file (*.txt)")
            if filename:
                self.save_map(filename)
        
        def new_map_callback():
            self.new_map()
            self.update()

        self.scenario['menu_map_load'].triggered.connect(load_map_callback)
        self.scenario['menu_map_save'].triggered.connect(save_map_callback)
        self.scenario['menu_map_new'].triggered.connect(new_map_callback)


        def load_conf_callback():
            filename, _ = QFileDialog.getOpenFileName(self, 'Open Conf', self.rtv.DATA_DIR, "Conf file (*.json)")
            
            if filename:
                self.scenario.load_conf(filename)
                self.reset()
                self.update()

        def load_default_conf_callback():
            self.scenario.load_default_conf()
            self.reset()
            self.update()

        def save_conf_callback():
            filename, _ = QFileDialog.getSaveFileName(self, 'Open File', self.rtv.DATA_DIR, "Conf file (*.json)")
            if filename:
                self.scenario.save_conf(filename)

        self.scenario['menu_conf_load'].triggered.connect(load_conf_callback)
        self.scenario['menu_conf_load_default'].triggered.connect(load_default_conf_callback)
        self.scenario['menu_conf_save'].triggered.connect(save_conf_callback)


    def drawSelectedArea(self):
        x1, y1 = self.selected_area[0]
        x2, y2 = self.converter.img_to_global(self.mouse_xy)
        Ax, Ay = min(x1, x2), max(y1, y2)
        Cx, Cy = max(x1, x2), min(y1, y2)

        iAx, iAy = self.converter.global_to_gl([Ax, Ay])
        iBx, iBy = self.converter.global_to_gl([Cx, Ay])
        iCx, iCy = self.converter.global_to_gl([Cx, Cy])
        iDx, iDy = self.converter.global_to_gl([Ax, Cy])

        self.gl.glColor3f(1, 0.5, 0.5)
        self.gl.glBegin(self.gl.GL_LINE_LOOP)
        self.gl.glVertex2f(iAx, iAy)
        self.gl.glVertex2f(iBx, iBy)
        self.gl.glVertex2f(iCx, iCy)
        self.gl.glVertex2f(iDx, iDy)
        self.gl.glEnd()


    def drawBackground(self):
        self.gl.glColor3f(1, 1, 1)
        ox, oy = self.rtv.background_offset
        for key in self.binded_data:
            id,  bb = self.binded_data[key]
            Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = bb
            iAx, iAy = self.converter.global_to_gl([Ax + ox, Ay + oy])
            iBx, iBy = self.converter.global_to_gl([Bx + ox, By + oy])
            iCx, iCy = self.converter.global_to_gl([Cx + ox, Cy + oy])
            iDx, iDy = self.converter.global_to_gl([Dx + ox, Dy + oy])
            cords = iAx, iAy, iBx, iBy, iCx, iCy, iDx, iDy
            self.drawImg(id, cords)
        self.gl.glBindTexture(self.gl.GL_TEXTURE_2D, 0)


    def drawBackgroundDebug(self):
        self.gl.glColor3f(1, 0, 0)
        for key in self.binded_data:
            id,  bb = self.binded_data[key]
            Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = bb
            iAx, iAy = self.converter.global_to_gl([Ax, Ay])
            iBx, iBy = self.converter.global_to_gl([Bx, By])
            iCx, iCy = self.converter.global_to_gl([Cx, Cy])
            iDx, iDy = self.converter.global_to_gl([Dx, Dy])
            self.gl.glBegin(self.gl.GL_LINE_LOOP)
            self.gl.glVertex2f(iAx, iAy)
            self.gl.glVertex2f(iBx, iBy)
            self.gl.glVertex2f(iCx, iCy)
            self.gl.glVertex2f(iDx, iDy)
            self.gl.glEnd()


    def drawMap(self):
        def paint_seq(_points, _color):
            r, g, b = _color
            self.gl.glColor3f(r, g, b)
            self.gl.glBegin(self.gl.GL_LINES)
            for (A, B) in _points:
                Ax, Ay = self.converter.global_to_gl([A.x, A.y])
                Bx, By = self.converter.global_to_gl([B.x, B.y])
                self.gl.glVertex2f(Ax, Ay)
                self.gl.glVertex2f(Bx, By)
            self.gl.glEnd()

        map = self.itf.map()
        lanes = map.lane

        for lane in lanes:
            points = lane.central_curve.segment[0].line_segment.point #reference trajectory
            points_zip  = list(zip(points[1:], points[:-1]))

            paint_seq(points_zip, (1, 0, 0))

            points = lane.left_boundary.curve.segment[0].line_segment.point #reference trajectory
            points_zip  = list(zip(points[1:], points[:-1]))

            paint_seq(points_zip, (0, 0, 0))

            points = lane.right_boundary.curve.segment[0].line_segment.point #reference trajectory
            points_zip  = list(zip(points[1:], points[:-1]))

            paint_seq(points_zip, (0, 0, 0))


    def drawRecorded(self):
        self.gl.glColor3f(0, 0, 1)
        for data in self.record_data:
            self.gl.glBegin(self.gl.GL_LINE_STRIP)
            for point in data:#points:
                Px, Py = self.converter.global_to_gl([point[0], point[1]])
                self.gl.glVertex2f(Px, Py)
            self.gl.glEnd()


    def drawImg(self, texture_id, cords):
        iAx, iAy, iBx, iBy, iCx, iCy, iDx, iDy = cords

        self.gl.glBindTexture(self.gl.GL_TEXTURE_2D, texture_id)

        self.gl.glBegin(self.gl.GL_TRIANGLES)

        self.gl.glTexCoord2f(0.0, 0.0)
        self.gl.glVertex2f(iDx, iDy)
        
        self.gl.glTexCoord2f(1.0, 0.0)
        self.gl.glVertex2f(iCx, iCy)
        
        self.gl.glTexCoord2f(1.0, 1.0)
        self.gl.glVertex2f(iBx, iBy)


        self.gl.glTexCoord2f(0.0, 0.0)
        self.gl.glVertex2f(iDx, iDy)

        self.gl.glTexCoord2f(1.0, 1.0)
        self.gl.glVertex2f(iBx, iBy)

        self.gl.glTexCoord2f(0.0, 1.0)
        self.gl.glVertex2f(iAx, iAy)
        
        self.gl.glEnd()


    def initializeGL(self):
        version_profile = QOpenGLVersionProfile()
        version_profile.setVersion(2, 0)
        self.gl = self.context().versionFunctions(version_profile)
        self.gl.initializeOpenGLFunctions()

        self.bind_area(self.rtv.background_roi)


    def paintGL(self):
        self.gl.glClear(self.gl.GL_COLOR_BUFFER_BIT | self.gl.GL_DEPTH_BUFFER_BIT)
        self.gl.glEnable(self.gl.GL_TEXTURE_2D) 

        if self.rtv.draw_background:
            self.drawBackground()
            #self.drawBackgroundDebug()
        
        if self.rtv.draw_records:
            self.drawRecorded()
        
        if self.rtv.draw_map:
            self.drawMap()

        if self.mouse_mode == MouseMode.SELECT_AREA and self.selected_area:
            self.drawSelectedArea()


    def bindTexture(self, img):
        img = Image.fromarray(img)
        w, h = img.size
        img_bytes = np.flipud(np.asarray(img)).tobytes()

        self.gl.glActiveTexture(self.gl.GL_TEXTURE0)
        texture_id = self.gl.glGenTextures(1)
        self.gl.glBindTexture(self.gl.GL_TEXTURE_2D, texture_id)
        self.gl.glPixelStorei(self.gl.GL_UNPACK_ALIGNMENT, 1)
        self.gl.glTexImage2D(self.gl.GL_TEXTURE_2D, 0, self.gl.GL_RGB, w, h, 0, self.gl.GL_RGB, self.gl.GL_UNSIGNED_BYTE, img_bytes)
        self.gl.glPixelStorei(self.gl.GL_UNPACK_ALIGNMENT, 4)
        self.gl.glTexParameterf(self.gl.GL_TEXTURE_2D, self.gl.GL_TEXTURE_MAG_FILTER, self.gl.GL_LINEAR)
        self.gl.glTexParameterf(self.gl.GL_TEXTURE_2D, self.gl.GL_TEXTURE_MIN_FILTER, self.gl.GL_LINEAR)
        return texture_id


        

            