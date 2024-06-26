
import PyQt5.QtCore
from utils.simulation import simulate
import json
import sys, os

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
                         "disable_obstacles": False, "colour_scheme": "Clean", "runtime_speed": 1, "show_muj_UI": False,
                         "enable_PID": True, "taskspace_name": "taskspace 1"}
        
        self.exportDict = {"dest_file": "./exportData/joint_data.csv", "start_time": 1, "end_time": 60, "save_type": "joint angles"}
        self.taskspace_Data, self.obstacle_id = {}, 0
        
        # ADD torque, position, velocity control. Do this actuatorgroupdisable element in mujoco
        self.confFile = "utils/simulationConf.json"

        self.setWindowTitle("Long Robot Configuration")
        dashboard_widget = QWidget()
        export_widget = QWidget()
        robot_setup_widget = QWidget()
        taskspace_widget = QWidget()
        help_widget = QWidget()
        
        
        self.title_font = PyQt5.QtGui.QFont()
        self.title_font.setBold(True)
        
        
        self.tabs = QTabWidget()
        self.tabs.addTab(dashboard_widget, "Dashboard")
        self.tabs.addTab(export_widget, "Export")
        self.tabs.addTab(robot_setup_widget, "Advanced Robot Settings")
        self.tabs.addTab(taskspace_widget, "Taskspace")
        self.tabs.addTab(help_widget, "Help")
        self.setCentralWidget(self.tabs)
        
        self.setup_dashboard(dashboard_widget)
        self.setup_export(export_widget)
        self.setup_robot_setup(robot_setup_widget)
        self.setup_taskspace(taskspace_widget)
        
    
    def add_obstacle_button(self):
        obstacle_box = QHBoxLayout()
        self.taskspace_Data[str(self.obstacle_id)] = [obstacle_box, {"Shape": "Circle", "X": 0, "Y": 0, "Z": 0, "Static": False, "alpha": 0, "beta": 0, "gamma": 0}]
        id_t = self.obstacle_id
        obstacle_type = QComboBox()
        obstacle_type.addItems(["Cylinder", "Square (not finished)"])
        obstacle_type.currentTextChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Shape", w)]))
        obstacle_box.addWidget(obstacle_type)
        
        x_coord_box, y_coord_box, z_coord_box = QHBoxLayout(), QHBoxLayout(), QHBoxLayout()
        x_coord_box.addWidget(QLabel("x"))
        y_coord_box.addWidget(QLabel("y"))
        z_coord_box.addWidget(QLabel("z"))
        x_coord, y_coord, z_coord = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        for i in [x_coord, y_coord, z_coord]:
            i.setMinimum(-1000)
            i.setMaximum(1000)
            i.setSingleStep(0.05)
            i.setMaximumWidth(55)
        
        
        x_coord.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("X", w)]))
        y_coord.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Y", w)]))
        z_coord.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Z", w)]))

        x_coord_box.addWidget(x_coord)
        y_coord_box.addWidget(y_coord)
        z_coord_box.addWidget(z_coord)
        
        obstacle_box.addLayout(x_coord_box)
        obstacle_box.addLayout(y_coord_box)
        obstacle_box.addLayout(z_coord_box)
        
        static_check_box = QCheckBox("Is Static")
        static_check_box.stateChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Static", not self.taskspace_Data[str(id)][1]["Static"])]))
        obstacle_box.addWidget(static_check_box)
 
        alpha_box, beta_box, gamma_box = QHBoxLayout(), QHBoxLayout(), QHBoxLayout()
        alpha_box.addWidget(QLabel(chr(945)))
        beta_box.addWidget(QLabel(chr(946)))
        gamma_box.addWidget(QLabel(chr(947)))
        alpha, beta, gamma = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        for i in [alpha, beta, gamma]:
            i.setMinimum(-360)
            i.setMaximum(360)
            i.setMaximumWidth(55)
        alpha.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("alpha", w)]))
        beta.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("beta", w)]))
        gamma.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("gamma", w)]))
        alpha_box.addWidget(alpha)
        beta_box.addWidget(beta)
        gamma_box.addWidget(gamma)
        
        obstacle_box.addLayout(alpha_box)
        obstacle_box.addLayout(beta_box)
        obstacle_box.addLayout(gamma_box)
        
        
        delete = QPushButton("-")
        delete.setMaximumWidth(30)
        delete.clicked.connect(lambda _, id=id_t: (
            self.clearLayout(self.taskspace_Data[str(id)][0]),
            self.taskspace_Data.pop(str(id))
        ))
        obstacle_box.addWidget(delete)
        
        self.manually_add.addLayout(obstacle_box)
        self.obstacle_id += 1
    
    def save_custom_obstacles_taskspace(self):
        filename = os.path.join("./MJCFS/taskspaces", self.confDict["taskspace_name"])
        with open(filename, "w+") as f:
            for i in self.taskspace_Data.values():
                print(i)
    def change_custom_taskspace_file(self, fname):
        self.confDict["taskspace_name"] = fname
        print(self.confDict)
            
    
    def setup_taskspace(self, taskspace_widget: QWidget):
        main_layout = QVBoxLayout()
        top_bar = QVBoxLayout()
        taskspace_title = QLabel("Create your taskspace here")
        taskspace_title.setFont(self.title_font)
        top_bar.addWidget(taskspace_title)
        notes = ["Instructions", f"- {chr(945)}, {chr(946)}, {chr(947)} are euler angles in degrees for the obstacle orientation", 
                 "- Static objects are locked in place and cannot move",
                 "- You must specify your taskspace name on the dashboard to use it after saving it"]
        for i in notes:
            bullet_box = QHBoxLayout()
            bullet_box.addSpacing(20)
            bullet_box.addWidget(QLabel(i))
            top_bar.addLayout(bullet_box)
        main_layout.addLayout(top_bar)
        main_layout.addStretch()
        
        taskspace_main = QHBoxLayout()
        GUI_taskspace = QVBoxLayout()
        GUI_taskspace.addWidget(QLabel("Click here to GUI it"))
        self.manually_add = QVBoxLayout()
        

        save_obstacles = QPushButton("Save Obstacles to file")
        save_obstacles.setMaximumWidth(300)
        save_obstacles.clicked.connect(self.save_custom_obstacles_taskspace)
        obstacle_save_name_box = QHBoxLayout()
        obstacle_save_name_box.addWidget(QLabel("Taskspace Name: "))
        obstacle_save_name = QLineEdit()
        obstacle_save_name.setPlaceholderText("Taskspace 1")
        obstacle_save_name.setMaximumWidth(250)
        obstacle_save_name.textChanged.connect(self.change_custom_taskspace_file)
        obstacle_save_name_box.addWidget(obstacle_save_name)
        obstacle_save_name_box.addStretch()
        
        
        self.manually_add.addWidget(save_obstacles)
        self.manually_add.addLayout(obstacle_save_name_box)
        add_obstacle = QPushButton("Add obstacle +")
        add_obstacle.setMaximumWidth(300)
        add_obstacle.clicked.connect(self.add_obstacle_button)
        self.manually_add.addWidget(add_obstacle)
        
        

        
        
        taskspace_main.addLayout(self.manually_add)
        taskspace_main.addStretch()
        taskspace_main.addLayout(GUI_taskspace)
        main_layout.addLayout(taskspace_main)
        
        taskspace_widget.setLayout(main_layout)
    
    
    def setup_help(self, help_widget: QWidget):
        pass
    
    def setup_robot_setup(self, setup_widget: QWidget):
        main_layout = QVBoxLayout()
        top_bar = QHBoxLayout()
        robot_setup_title = QLabel("More advanced robot settings (optional)")
        robot_setup_title.setFont(self.title_font)
        top_bar.addWidget(robot_setup_title)
        main_layout.addLayout(top_bar)
        
        main_setup = QHBoxLayout()
        main_setup.addWidget(QLabel("UNFINISHED"))
        main_layout.addLayout(main_setup)
        setup_widget.setLayout(main_layout)        

    def setup_export(self, export: QWidget):
        main_layout = QVBoxLayout()
        top_bar = QHBoxLayout()
        export_title = QLabel("Export Data ")
        export_title.setFont(self.title_font)
        top_bar.addWidget(export_title)
        main_layout.addLayout(top_bar)
        
        main_section = QHBoxLayout()
        settings_section = QVBoxLayout()
        
        # Save Type
        save_type_box = QHBoxLayout()
        save_type_box.addWidget(QLabel("What do you want to export?"))
        save_type = QComboBox()
        save_type.addItems(["joint angles", "joint torques", "joint positions (x y z)"])
        save_type.currentTextChanged.connect(self.change_save_type)
        save_type_box.addWidget(save_type)
        settings_section.addLayout(save_type_box)
        
        # Start time
        start_time_box = QHBoxLayout()
        start_time_box.addWidget(QLabel("Start time (seconds): "))
        start_time = QSpinBox()
        start_time.setMinimum(0)
        start_time.setValue(int(self.exportDict["start_time"]))
        start_time.setMaximum(10000000)
        start_time.valueChanged.connect(self.change_start_time)
        start_time_box.addWidget(start_time)
        settings_section.addLayout(start_time_box)
        
        # End time
        end_time_box = QHBoxLayout()
        end_time_box.addWidget(QLabel("End time (seconds): "))
        end_time = QSpinBox()
        end_time.setMinimum(1)
        end_time.setMaximum(10000000)
        end_time.setValue(int(self.exportDict["end_time"]))
        end_time.valueChanged.connect(self.change_end_time)
        end_time_box.addWidget(end_time)
        settings_section.addLayout(end_time_box)
         
    
        # File edit
        file_dest_box = QHBoxLayout()
        file_dest_box.addWidget(QLabel("Save to: "))
        file_dest = QLineEdit(self.exportDict["dest_file"])
        file_dest.textEdited.connect(self.change_export_path)
        file_dest_box.addWidget(file_dest)
        settings_section.addLayout(file_dest_box)
        
        main_section.addLayout(settings_section)
        main_layout.addLayout(main_section)
        
        export.setLayout(main_layout)
     
    def change_export_path(self, filename):
        self.exportDict["dest_file"] = filename 
        print(self.exportDict)
     
    def change_save_type(self, save_type):
        self.exportDict["save_type"] = save_type
        print(self.exportDict)
    
    def change_start_time(self, start):
        self.exportDict["start_time"] = start
        
    def change_end_time(self, end):
        self.exportDict["end_time"] = end
       
    def setup_dashboard(self, dashboard):
        # Top Bar setup
        title = QVBoxLayout()
        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel("VERSION 0.0001"))
        logo_holder = QLabel()
        logo_holder.setPixmap(QPixmap('media/logo.png'))
        top_bar.addWidget(logo_holder)
        title.addLayout(top_bar)
        main_layout = QHBoxLayout()
       
        # Sections
        self.left = QVBoxLayout()
        self.middle = QVBoxLayout()
        self.right = QVBoxLayout()
        
        # Left Section
        self.setup_settings()

        # Middle Section
        self.setup_preview()
        
        # Right Section
        self.setup_controls()

        # Main layout
        main_layout.addLayout(self.left)
        main_layout.addSpacing(50)
        main_layout.addLayout(self.middle)
        main_layout.addSpacing(50)
        main_layout.addLayout(self.right)

        title.addSpacing(20)
        title.addLayout(main_layout)
        dashboard.setLayout(title)
    
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
        
    def toggle_pid_control(self):
        self.confDict["enable_PID"] = not self.confDict["enable_PID"]
    
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
        
        # PID controller
        pid_controller_box = QCheckBox("Disable PID Controller (movement)")
        pid_controller_box.stateChanged.connect(self.toggle_pid_control)
        self.left.addWidget(pid_controller_box)
        
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




app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()



#simulate("./utils/simulationConf.json")