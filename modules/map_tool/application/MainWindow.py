from PyQt5.QtWidgets import QWidget, QMainWindow, QLineEdit, QFileDialog, QPushButton, \
                            QHBoxLayout, QVBoxLayout, QGridLayout, QLabel, QTableWidget, QAction
from PyQt5.QtCore    import Qt, QSize

from application.MapField import MapField


class MainWindow(QMainWindow):

    def __init__(self, scenario):
        super().__init__()
        self.scenario = scenario
        
        main_widget = QWidget()

        self.menubar()

        top_gbox = self.top_gbox()
        self.scenario['map_field'] = MapField(self.scenario, self)

        gbox = QGridLayout()
        gbox.addLayout(top_gbox, 0, 1)
        gbox.addWidget(self.scenario['map_field'], 1, 1)

        main_widget.setLayout(gbox)
        self.setCentralWidget(main_widget)
        #self.setFixedWidth(1800)
        #self.showMaximized() 


    def top_gbox(self):
        gbox = QGridLayout()
        self.scenario['cur_pos_img'] = QLineEdit()
        self.scenario['cur_pos_img'].setReadOnly(True)
        pos_img_desc = QLabel("Image position (pixels)")

        self.scenario['cur_pos_global'] = QLineEdit()
        self.scenario['cur_pos_global'].setReadOnly(True)
        pos_global_desc = QLabel("Global position (UTM)")

        self.scenario['cur_pos_lonlat'] = QLineEdit()
        self.scenario['cur_pos_lonlat'].setReadOnly(True)
        pos_lonlat_desc = QLabel("Image position (lonlat)")

        gbox.addWidget(pos_img_desc, 0, 0)
        gbox.addWidget(self.scenario['cur_pos_img'], 0, 1)
        gbox.addWidget(pos_global_desc, 0, 2)
        gbox.addWidget(self.scenario['cur_pos_global'], 0, 3)
        gbox.addWidget(pos_lonlat_desc, 0, 4)
        gbox.addWidget(self.scenario['cur_pos_lonlat'], 0, 5)

        self.scenario['background_offset'] = QLineEdit()
        wg_background_offset = QLabel("Background offset (UTM)")

        self.scenario['offset_global'] = QLineEdit()
        wg_offset_global = QLabel("Global offset (UTM)")

        self.scenario['scale'] = QLineEdit()
        wg_scale= QLabel("Image scale")

        gbox.addWidget(wg_background_offset, 1, 0)
        gbox.addWidget(self.scenario['background_offset'], 1, 1)
        gbox.addWidget(wg_offset_global, 1, 2)
        gbox.addWidget(self.scenario['offset_global'], 1, 3)
        gbox.addWidget(wg_scale, 1, 4)
        gbox.addWidget(self.scenario['scale'], 1, 5)

        return gbox
    

    def toolbar(self):
        self.scenario['action1'] = QAction('123')

        self.scenario['toolbar'] = self.addToolBar('Tools')
        self.scenario['toolbar'].addAction(self.scenario['action1'])


    def menubar(self):
        self.scenario['menu'] = self.menuBar()


        #Conf: load save load_default clear (window name ~ filename)
        self.scenario['menu_conf'] = self.scenario['menu'].addMenu('&Conf')
        
        self.scenario['menu_conf_load'] = QAction('Load', self)
        self.scenario['menu_conf'].addAction(self.scenario['menu_conf_load'])
        self.scenario['menu_conf_save'] = QAction('Save', self)
        self.scenario['menu_conf'].addAction(self.scenario['menu_conf_save'])
        self.scenario['menu_conf_load_default'] = QAction('Load Default', self)
        self.scenario['menu_conf'].addAction(self.scenario['menu_conf_load_default'])


        #Map: load save new (mb maket?) (gen using apollo scripts)
        self.scenario['menu_map'] = self.scenario['menu'].addMenu('&Map')

        self.scenario['menu_map_load'] = QAction('Load', self)
        self.scenario['menu_map'].addAction(self.scenario['menu_map_load'])
        self.scenario['menu_map_save'] = QAction('Save', self)
        self.scenario['menu_map'].addAction(self.scenario['menu_map_save'])
        self.scenario['menu_map_new'] = QAction('New', self)
        self.scenario['menu_map'].addAction(self.scenario['menu_map_new'])
        
        
        #Mode: preview edit map
        self.scenario['menu_mode'] = self.scenario['menu'].addMenu('&Edit Mode')

        self.scenario['menu_mode_preview'] = QAction('Preview', self)
        self.scenario['menu_mode'].addAction(self.scenario['menu_mode_preview'])
        self.scenario['menu_mode_edit_map'] = QAction('Edit map', self)
        self.scenario['menu_mode'].addAction(self.scenario['menu_mode_edit_map'])


        #Draw settings: chechbox whitch ones to draw
        self.scenario['menu_draw_settings'] = self.scenario['menu'].addMenu('&Draw Settings')

        self.scenario['menu_draw_map'] = QAction('Draw map', self)
        self.scenario['menu_draw_map'].setCheckable(True)
        #self.scenario['menu_draw_map'].setChecked(True)
        self.scenario['menu_draw_settings'].addAction(self.scenario['menu_draw_map'])

        self.scenario['menu_draw_records'] = QAction('Draw records', self)
        self.scenario['menu_draw_records'].setCheckable(True)
        self.scenario['menu_draw_settings'].addAction(self.scenario['menu_draw_records'])

        self.scenario['menu_draw_background'] = QAction('Draw backround', self)
        self.scenario['menu_draw_background'].setCheckable(True)
        self.scenario['menu_draw_settings'].addAction(self.scenario['menu_draw_background'])


        #Backround selection: select area (may be called multiple times in different areas) clear
        self.scenario['menu_background_selection'] = self.scenario['menu'].addMenu('&Backround selection')

        self.scenario['menu_background_add'] = QAction('Add region', self)
        self.scenario['menu_background_selection'].addAction(self.scenario['menu_background_add'])
        
        self.scenario['menu_background_clear'] = QAction('Clear background', self)
        self.scenario['menu_background_selection'].addAction(self.scenario['menu_background_clear'])


        #Records add clear (menu with which one enables)
        self.scenario['menu_records'] = self.scenario['menu'].addMenu('&Records')

        self.scenario['menu_records_add_apollo'] = QAction('Add apollo record', self)
        self.scenario['menu_records'].addAction(self.scenario['menu_records_add_apollo'])

        self.scenario['menu_records_add_gkv'] = QAction('Add GKV record', self)
        self.scenario['menu_records'].addAction(self.scenario['menu_records_add_gkv'])
        
        self.scenario['menu_records_clear'] = QAction('Clear', self)
        self.scenario['menu_records'].addAction(self.scenario['menu_records_clear'])

        #submenu for record list
        self.scenario['menu_submenu_records'] = self.scenario['menu_records'].addMenu('Record list')
        

        #Apollo: gen apollo map add car tracking centralize on car
        self.scenario['menu_apollo'] = self.scenario['menu'].addMenu('&Apollo')

        self.scenario['menu_apollo_gen_map'] = QAction('Generate apollo map', self)
        self.scenario['menu_apollo'].addAction(self.scenario['menu_apollo_gen_map'])


        self.scenario['menu_submenu_apollo_car'] = self.scenario['menu_apollo'].addMenu('Car')
        
        self.scenario['menu_apollo_car_tracking'] = QAction('Enable car tracking', self)
        self.scenario['menu_apollo_car_tracking'].setCheckable(True)
        self.scenario['menu_submenu_apollo_car'].addAction(self.scenario['menu_apollo_car_tracking'])

        self.scenario['menu_apollo_centralize_on_car'] = QAction('Centralize on car', self)
        self.scenario['menu_submenu_apollo_car'].addAction(self.scenario['menu_apollo_centralize_on_car'])
        
        # Arrow, that pointing to far objects?
        # Simple way to select records to draw
        # Records in submenu ?