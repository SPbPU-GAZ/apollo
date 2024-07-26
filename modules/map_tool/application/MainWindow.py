from PyQt5.QtWidgets import QWidget, QMainWindow, QLineEdit, QFileDialog, QPushButton, \
                            QHBoxLayout, QVBoxLayout, QGridLayout, QLabel, QTableWidget
from PyQt5.QtCore    import QSize

from application.MapField import MapField


class MainWindow(QMainWindow):

    def __init__(self, scenario):
        super().__init__()
        self.scenario = scenario
        
        main_widget = QWidget()
        top_gbox = self.top_gbox()
        bottom_gbox = self.left_gbox()
        #hat_box

        self.scenario['map_field'] = MapField(self.scenario, self)

        gbox = QGridLayout()
        gbox.addLayout(top_gbox, 0, 1)
        gbox.addWidget(self.scenario['map_field'], 1, 1)
        gbox.addLayout(bottom_gbox, 0, 0)

        main_widget.setLayout(gbox)
        self.setCentralWidget(main_widget)
        self.setFixedWidth(1800)
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

        self.scenario['offset_img'] = QLineEdit()
        wg_offset_img = QLabel("Image offset (pixels)")

        self.scenario['offset_global'] = QLineEdit()
        wg_offset_global = QLabel("Global offset (UTM)")

        self.scenario['scale'] = QLineEdit()
        wg_scale= QLabel("Image scale")

        gbox.addWidget(wg_offset_img, 1, 0)
        gbox.addWidget(self.scenario['offset_img'], 1, 1)
        gbox.addWidget(wg_offset_global, 1, 2)
        gbox.addWidget(self.scenario['offset_global'], 1, 3)
        gbox.addWidget(wg_scale, 1, 4)
        gbox.addWidget(self.scenario['scale'], 1, 5)

        return gbox


    def left_gbox(self):
        gbox = QGridLayout()

        conf_filename_label = QLabel("Conf file: ")
        self.scenario['conf_filename'] = QLineEdit()
        self.scenario['conf_filename'].setReadOnly(True)
        #self.scenario['conf_filename'].setFixedWidth(100)
        self.scenario['conf_load'] = QPushButton("load")
        self.scenario['conf_save'] = QPushButton("save")

        gbox.addWidget(conf_filename_label, 0, 0)
        gbox.addWidget(self.scenario['conf_filename'], 0, 1)
        gbox.addWidget(self.scenario['conf_load'], 0, 2) #call only here
        gbox.addWidget(self.scenario['conf_save'], 0, 3) #save that already had been selected
    
        map_filename_label = QLabel("Map file: ")
        self.scenario['map_filename'] = QLineEdit()
        self.scenario['map_filename'].setReadOnly(True)
        #self.scenario['map_filename'].setFixedWidth(100)
        self.scenario['map_load'] = QPushButton("load")
        self.scenario['map_save'] = QPushButton("save")
        
        gbox.addWidget(map_filename_label, 1, 0)
        gbox.addWidget(self.scenario['map_filename'], 1, 1)
        gbox.addWidget(self.scenario['map_load'], 1, 2) #call only here
        gbox.addWidget(self.scenario['map_save'], 1, 3) #save that already had been selected

        #table = QTableWidget()

        return gbox
    