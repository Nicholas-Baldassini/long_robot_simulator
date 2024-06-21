
from utils.simulation import simulate
import json
import sys

import PyQt5
from PyQt5.QtWidgets import QApplication, QLabel
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from PyQt5.QtWidgets import *
from utils.createTaskspace import start_taskspace_creator

sys.path.append("utils/")


from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap
#import PyQt5.QtGui

import sys


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.confDict = {"robot_lock": True, "num_joints": 30, "joint_spacing": 0.05, "gravity": True, 
                         "disable_obstacles": False, "colour_scheme": "Clean", "runtime_speed": 1, "show_muj_UI": False}
        
        # ADD torque, position, velocity control. Do this actuatorgroupdisable element in mujoco
        self.confFile = "utils/simulationConf.json"

        self.setWindowTitle("Long Robot Configuration")
        container = QWidget()
        
        self.title_font = PyQt5.QtGui.QFont()
        self.title_font.setBold(True)
        
        # Top Bar setup
        title = QVBoxLayout()
        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel("VERSION 0.0001"))
        logo_holder = QLabel()
        logo_holder.setPixmap(QPixmap('media/logo.png'))
        top_bar.addWidget(logo_holder)
        title.addLayout(top_bar)
        main_layout = QHBoxLayout()
       
       
        self.left = QVBoxLayout()
        self.middle = QVBoxLayout()
        self.right = QVBoxLayout()
        
        # Left Section
        self.setup_settings()

        # Middle Section
        self.setup_preview()
        
        # Right Section
        self.setup_controls()


        main_layout.addLayout(self.left)
        main_layout.addSpacing(50)
        main_layout.addLayout(self.middle)
        main_layout.addSpacing(50)
        main_layout.addLayout(self.right)

        title.addSpacing(20)
        title.addLayout(main_layout)
        container.setLayout(title)
        
        #self.table_widget = MyTableWidget(self)
        #self.setCentralWidget(self.table_widget)
        self.setCentralWidget(container)
        
        
    
    def save_file(self):
        with open(self.confFile, "w") as f:
            #f.write(json.dumps(self.confDict))
            json.dump(self.confDict, f, indent=4)
        #print(json.dumps(self.confDict))
        #print(self.confDict)
        self.setup_preview()
    
    def start_simulation(self):
        from multiprocessing import Process
        p = Process(target=simulate, args=(self.confFile,))
        p.start()
        #p.join()
        #simulate(self.confFile)
        print("Started Simulation")
        
    def create_Taskspace(self):
        start_taskspace_creator()
        print("Taskspace Created")
    
    def toggle_movement(self):
        self.confDict["robot_lock"] = not self.confDict["robot_lock"]   
        #self.setup_preview()
    
    def toggle_gravity(self):
        self.confDict["gravity"] = not self.confDict["gravity"]
        #self.setup_preview()
        
    def toggle_muj_ui(self):
        self.confDict["show_muj_UI"] = not self.confDict["show_muj_UI"]
        
        #self.setup_preview()
    
    def toggle_obstacles(self):
        self.confDict["disable_obstacles"] = not self.confDict["disable_obstacles"]
        #self.setup_preview()

    def update_joints_num(self, num):
        self.confDict["num_joints"] = num
        #self.setup_preview()
        
    def update_joints_spacing(self, num):
        self.confDict["joint_spacing"] = num / 1000
        #self.setup_preview()

    def update_runtime_speed(self, num):
        self.confDict["runtime_speed"] = num / 100
        #self.setup_preview()
    
    def update_colour_scheme(self, val):
        print(val)
        self.confDict["colour_scheme"] = val
        #self.setup_preview()      
        
    
    def setup_preview(self):
        self.clearLayout(self.middle)
        sec_title = QLabel("Current Saved Configuration")
        sec_title.setFont(self.title_font)
        self.middle.addWidget(sec_title)
        for i in self.confDict:
            pair = QHBoxLayout()
            pair.addWidget(QLabel(i))
            pair.addWidget(QLabel(": " + str(self.confDict[i])))
            self.middle.addLayout(pair)
            
    def setup_settings(self):
        sec_title = QLabel("Change Simulation Settings")
        sec_title.setFont(self.title_font)
        self.left.addWidget(sec_title)
        
        # Movement
        check = QCheckBox("Lock robot in place")
        check.stateChanged.connect(self.toggle_movement)
        self.left.addWidget(check)
        
        # Joint number
        joints_num_box = QHBoxLayout()
        joints_num_box.addWidget(QLabel("Joint Number: "))
        joints_num = QSpinBox()
        joints_num.setMinimum(2)
        joints_num.setValue(int(self.confDict["num_joints"]))
        joints_num.setMaximum(1000)
        joints_num.valueChanged.connect(self.update_joints_num)
        joints_num_box.addWidget(joints_num)
        self.left.addLayout(joints_num_box)
        
        # Joint Spacing
        joint_spacinggth_box = QHBoxLayout()
        joint_spacinggth_box.addWidget(QLabel("Joint Spacing:  "))
        joint_spacinggth = QSpinBox()
        joint_spacinggth.setMinimum(1)
        joint_spacinggth.setMaximum(1000)
        joint_spacinggth.setSingleStep(5)
        joint_spacinggth.setValue(int(self.confDict["joint_spacing"] * 1000))
        joint_spacinggth.valueChanged.connect(self.update_joints_spacing)
        joint_spacinggth_box.addWidget(joint_spacinggth)
        self.left.addLayout(joint_spacinggth_box)
        
        # Gravity
        check_grav = QCheckBox("Disable Gravity")
        check_grav.stateChanged.connect(self.toggle_gravity)
        self.left.addWidget(check_grav)
        
        # Obstacles Enablle
        check_obstacles = QCheckBox("Disable Obstacles")
        check_obstacles.stateChanged.connect(self.toggle_obstacles)
        self.left.addWidget(check_obstacles)
        
        # Runtime speed
        runtime_speed_box = QHBoxLayout()
        runtime_speed_box.addWidget(QLabel("Runtime Speed Delay %: "))
        runtime_speed = QSpinBox()
        runtime_speed.setMinimum(1)
        runtime_speed.setMaximum(500)
        runtime_speed.setValue(100)
        runtime_speed.setSingleStep(5)
        runtime_speed.valueChanged.connect(self.update_runtime_speed)
        runtime_speed_box.addWidget(runtime_speed)
        self.left.addLayout(runtime_speed_box)
        
        # Colour Scheme
        colour_scheme_box = QHBoxLayout()
        colour_scheme_box.addWidget(QLabel("Colour Scheme: "))
        colour_scheme = QComboBox()
        colour_scheme.addItems(["Clean", "Cinematic (slower)"])
        colour_scheme.currentTextChanged.connect(self.update_colour_scheme)
        colour_scheme_box.addWidget(colour_scheme)
        self.left.addLayout(colour_scheme_box)
        
        # Mujoco UI
        muj_ui = QCheckBox("Show Mujoco UI")
        muj_ui.stateChanged.connect(self.toggle_muj_ui)
        self.left.addWidget(muj_ui)
        
    def setup_controls(self):
        sec_title = QLabel("Controls")
        sec_title.setFont(self.title_font)
        self.right.addWidget(sec_title)
        
        save_button = QPushButton("Save Configuration File")
        save_button.clicked.connect(self.save_file)
        self.right.addWidget(save_button)
        
        start_button = QPushButton("Start Simulation")
        start_button.clicked.connect(self.start_simulation)
        self.right.addWidget(start_button)
        
        taskspace_button = QPushButton("Create Taskspace")
        taskspace_button.clicked.connect(self.create_Taskspace)
        self.right.addWidget(taskspace_button)
        

    def clearLayout(self, layout):
        if layout is not None:
            while layout.count():
                child = layout.takeAt(0)
                if child.widget() is not None:
                    child.widget().deleteLater()
                elif child.layout() is not None:
                    self.clearLayout(child.layout())



class MyTableWidget(QWidget):
    
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout(self)
        
        # Initialize tab screen
        self.tabs = QTabWidget()
        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tabs.resize(300,200)
        
        # Add tabs
        self.tabs.addTab(self.tab1,"Tab 1")
        self.tabs.addTab(self.tab2,"Tab 2")
        
        # Create first tab
        self.tab1.layout = QVBoxLayout(self)
        self.pushButton1 = QPushButton("PyQt5 button")
        self.tab1.layout.addWidget(self.pushButton1)
        self.tab1.setLayout(self.tab1.layout)
        
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)
    from PyQt5.QtCore import pyqtSlot
    @pyqtSlot()
    def on_click(self):
        print("\n")
        for currentQTableWidgetItem in self.tableWidget.selectedItems():
            print(currentQTableWidgetItem.row(), currentQTableWidgetItem.column(), currentQTableWidgetItem.text())



app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()



#simulate("./utils/simulationConf.json")