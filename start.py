
import PyQt5.QtCore
import PyQt5.QtWidgets
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

        # ADD torque, position, velocity control. Do this actuatorgroupdisable element in mujoco
        self.confFile = "utils/simulationConf.json"
        
        # Main configuration settings dictionary
        self.confDict = {"robot_lock": True, "num_joints": 30, "joint_spacing": 0.05, "gravity": True, 
                         "disable_obstacles": False, "colour_scheme": "Clean", "runtime_speed": 1, "show_muj_UI": False,
                         "enable_PID": True, "taskspace_name": "taskspace.conf", "motionplan_name": "Plan_1"}
        # Attempt to load in previoulsy saved file
        try:
            conf = json.load(open(self.confFile))
            for i in conf.items():
                self.confDict[i[0]] = i[1]
        except:
            print("Could not load in old configuration file")
        
        # Dictionary for exporting data
        self.exportDict = {"dest_file": "./exportData/joint_data.csv", "start_time": 1, "end_time": 60, "save_type": "joint angles"}
        
        # Taskspace creation dictionary
        self.taskspace_Data, self.obstacle_id, self.taskspace_page_filename = {}, 0, self.confDict["taskspace_name"]
        
        # Motion plan dictionary
        self.motionPlan_Data, self.motion_id, self.motion_plan_page_filename = {}, 0, self.confDict["motionplan_name"]
        
        # Robot settings dictionary
        self.advancedSettings = {"thickness": 0.035}
        

        # Tabs on main window
        self.setWindowTitle("Long Robot Configuration")
        dashboard_widget = QWidget()
        export_widget = QWidget()
        robot_setup_widget = QWidget()
        taskspace_widget = QWidget()
        help_widget = QWidget()
        motion_plan_widget = QWidget()
        
        # Bold font
        self.title_font = PyQt5.QtGui.QFont()
        self.title_font.setBold(True)
        
        # Add tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(dashboard_widget, "Dashboard")
        self.tabs.addTab(export_widget, "Export")
        self.tabs.addTab(robot_setup_widget, "Advanced Robot Settings")
        self.tabs.addTab(taskspace_widget, "Taskspace")
        self.tabs.addTab(motion_plan_widget, "Motion Plan")
        self.tabs.addTab(help_widget, "Help")
        self.setCentralWidget(self.tabs)
        
        # Setup each tab seperately
        self.setup_dashboard(dashboard_widget)
        self.setup_export(export_widget)
        self.setup_robot_setup(robot_setup_widget)
        self.setup_taskspace(taskspace_widget)
        self.setup_motionplan(motion_plan_widget)
        

    def add_manual_plan_point(self):
        """Adding a point to the custom motion plan"""
        point_box = QHBoxLayout()
        self.motionPlan_Data[str(self.motion_id)] = [point_box, {"torque": 0.5, "base_speed": 0.5, "start_time": 0, "end_time": 5, "interpolation_speed": 0.5}]
        id_t = self.motion_id
        
        # Torque of SS TDCR
        torque_box = QHBoxLayout()
        torque_box.addWidget(QLabel("Torque: "))
        torque = QDoubleSpinBox()
        torque.setSingleStep(0.05)
        torque.setValue(self.motionPlan_Data[str(self.motion_id)][1]["torque"])
        torque.setDecimals(3)
        torque.textChanged.connect(lambda w, id=id_t: self.motionPlan_Data[str(id)][1].update([("torque", w)]))
        torque_box.addWidget(torque)
        point_box.addLayout(torque_box)
        
        # Base_speed value
        speed_val_box = QHBoxLayout()
        speed_val_box.addWidget(QLabel("Linear Speed: "))
        speed_val = QDoubleSpinBox()
        speed_val.setSingleStep(0.5)
        speed_val.setDecimals(3)
        speed_val.setValue(self.motionPlan_Data[str(self.motion_id)][1]["base_speed"])
        speed_val.textChanged.connect(lambda w, id=id_t: self.motionPlan_Data[str(id)][1].update([("base_speed", w)]))
        speed_val_box.addWidget(speed_val)
        point_box.addLayout(speed_val_box)
        
        # Start_time
        start_time_box = QHBoxLayout()
        start_time_box.addWidget(QLabel("Start: "))
        start_time = QDoubleSpinBox()
        start_time.setSingleStep(0.5)
        # - Set the start time to last object end time
        last_end_time = 0
        if self.motionPlan_Data:
            most_recent_id = 0
            for i in self.motionPlan_Data.items():
                if int(i[0]) != id_t:
                    most_recent_id = max(most_recent_id, int(i[0]))
            if most_recent_id != id_t:
                last_end_time = float(self.motionPlan_Data[str(most_recent_id)][1]["end_time"])
            print(most_recent_id, id_t)
        start_time.setValue(last_end_time)
        # - Set the start time to last object end time
        start_time.textChanged.connect(lambda w, id=id_t: self.motionPlan_Data[str(id)][1].update([("start_time", w)]))
        start_time_box.addWidget(start_time)
        point_box.addLayout(start_time_box)
        
        # End_time
        end_time_box = QHBoxLayout()
        end_time_box.addWidget(QLabel("End: "))
        end_time = QDoubleSpinBox()
        end_time.setValue(self.motionPlan_Data[str(self.motion_id)][1]["end_time"])
        end_time.setSingleStep(0.5)
        end_time.textChanged.connect(lambda w, id=id_t: self.motionPlan_Data[str(id)][1].update([("end_time", w)]))
        end_time_box.addWidget(end_time)
        point_box.addLayout(end_time_box)
        
        #Interpolation speed
        interpolation_speed_box = QHBoxLayout()
        interpolation_speed_box.addWidget(QLabel("Interpolation: "))
        #
        #
        #
        point_box.addLayout(interpolation_speed_box)
        
        # Delete button
        delete = QPushButton("-")
        delete.setMaximumWidth(30)
        delete.clicked.connect(lambda _, id=id_t: (
            self.clearLayout(self.motionPlan_Data[str(id)][0]),
            self.motionPlan_Data.pop(str(id))
        ))
        point_box.addWidget(delete)
        
        
        
        self.manually_add_plan.addLayout(point_box)
        self.motion_id += 1
    
    def add_obstacle_button(self):
        """Adding an obstacle to the taskspace page, each time you click "add object" """
        obstacle_box = QHBoxLayout()
        
        # Data each obstacle holds
        self.taskspace_Data[str(self.obstacle_id)] = [obstacle_box, {"Shape": "Circle", "X": 0, "Y": 0, "Z": 0, "Static": False, "alpha": 0, "beta": 0, "gamma": 0, "Size": 0.5}]
        # Each obstacle has its own ID to keep track of it
        id_t = self.obstacle_id
        
        # Object type combo box
        obstacle_type = QComboBox()
        obstacle_type.setEditable(True)
        obstacle_type.addItems(["Cylinder", "Square (not finished)", "STLS_dir/teaCup.stl"])
        obstacle_type.currentTextChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Shape", w)]))
        obstacle_box.addWidget(obstacle_type)
        
        # The XYZ coordinate boxes & inputs
        x_coord_box, y_coord_box, z_coord_box = QHBoxLayout(), QHBoxLayout(), QHBoxLayout()
        x_coord_box.addWidget(QLabel("x"))
        y_coord_box.addWidget(QLabel("y"))
        z_coord_box.addWidget(QLabel("z"))
        x_coord, y_coord, z_coord = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        for i in [x_coord, y_coord, z_coord]:
            i.setMinimum(-1000)
            i.setMaximum(1000)
            i.setSingleStep(0.5)
            i.setMaximumWidth(50)
        
        # XYZ, each connect is connected to its own indidvidual callback with its respective ID
        x_coord.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("X", w)]))
        y_coord.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Y", w)]))
        z_coord.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Z", w)]))

        # Adding XYZ input to box
        x_coord_box.addWidget(x_coord)
        y_coord_box.addWidget(y_coord)
        z_coord_box.addWidget(z_coord)
        
        # Adding XYZ to object box
        obstacle_box.addLayout(x_coord_box)
        obstacle_box.addLayout(y_coord_box)
        obstacle_box.addLayout(z_coord_box)
        
        # Static check mark
        static_check_box = QCheckBox("Is Static")
        static_check_box.setMaximumWidth(70)
        static_check_box.stateChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Static", not self.taskspace_Data[str(id)][1]["Static"])]))
        obstacle_box.addWidget(static_check_box)
 
        # Euler angles 
        alpha_box, beta_box, gamma_box = QHBoxLayout(), QHBoxLayout(), QHBoxLayout()
        alpha_box.addWidget(QLabel(chr(945)))
        beta_box.addWidget(QLabel(chr(946)))
        gamma_box.addWidget(QLabel(chr(947)))
        alpha, beta, gamma = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        for i in [alpha, beta, gamma]:
            i.setMinimum(-360)
            i.setMaximum(360)
            i.setMaximumWidth(50)
            i.setSingleStep(5)
        alpha.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("alpha", w)]))
        beta.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("beta", w)]))
        gamma.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("gamma", w)]))
        alpha_box.addWidget(alpha)
        beta_box.addWidget(beta)
        gamma_box.addWidget(gamma)
        obstacle_box.addLayout(alpha_box)
        obstacle_box.addLayout(beta_box)
        obstacle_box.addLayout(gamma_box)
        
        # Size box
        ob_size_box = QHBoxLayout()
        ob_size_box.addWidget(QLabel("Size"))
        ob_size = QDoubleSpinBox()
        ob_size.setMinimum(0.00001)
        ob_size.setValue(0.5)
        ob_size.setDecimals(3)
        ob_size.setSingleStep(0.001)
        ob_size.setMaximum(50)
        ob_size.setMaximumWidth(85)
        ob_size.textChanged.connect(lambda w, id=id_t: self.taskspace_Data[str(id)][1].update([("Size", w)]))
        ob_size_box.addWidget(ob_size)
        obstacle_box.addLayout(ob_size_box)
        
        # Delete button
        delete = QPushButton("-")
        delete.setMaximumWidth(30)
        delete.clicked.connect(lambda _, id=id_t: (
            self.clearLayout(self.taskspace_Data[str(id)][0]),
            self.taskspace_Data.pop(str(id))
        ))
        obstacle_box.addWidget(delete)
        
        # Add box to the taskspace screen
        self.manually_add.addLayout(obstacle_box)
        # Increment ID
        self.obstacle_id += 1
    
    def save_custom_obstacles_taskspace(self):
        """Callback when saving file in taskspace screen"""
        #self.show_alert("Hello world", level="Critical")
        filename = os.path.join("./MJCFS/taskspaces", self.taskspace_page_filename)
        with open(filename, "w+") as f:
            for v in self.taskspace_Data.values():
                i = v[1]
                #line = f"{i['Shape']} {i['Size']} {i['X']} {i['Y']} {i['Static']}\n"
                #shape, scale, y, x, z, static, alpha, beta, gamma
                line = f"{i['Shape']} {i['Size']} {i['Y']} {i['X']} {i['Z']} {i['Static']} {i['alpha']} {i['beta']} {i['gamma']}\n"
                f.write(line)
                
    
    def save_manual_motionplan(self):
        """Callback when saving file in taskspace screen"""
        filename = os.path.join("./MotionPlans", self.motion_plan_page_filename)
        with open(filename, "w+") as f:
            for p in self.motionPlan_Data.values():
                
                f.write(str(p))
         
    def change_custom_taskspace_file(self, fname):
        """Taskspace screen change filename tasksapce objects are saved too"""
        self.taskspace_page_filename = fname

    def change_setting_taskspace_name(self, fname):
        """Taskspace filename that is loaded in on main dashboard screen"""
        self.confDict["taskspace_name"] = fname
    
    def change_manual_motionplan_file(self, fname):
        """Manual Motion plan screen change filename"""
        self.motion_plan_page_filename = fname
    
    def change_setting_motionplan_name(self, fname):
        """Motion plan filename that is loaded in on main dashboard screen"""
        self.confDict["motionplan_name"] = fname
        
    
            
    
    def setup_taskspace(self, taskspace_widget: QWidget):
        """Taskspace creation screen setup function"""
        
        # Entire page layout
        main_layout = QVBoxLayout()
        # Top bar & title layout
        top_bar = QVBoxLayout()
        taskspace_title = QLabel("Create your taskspace here")
        taskspace_title.setFont(self.title_font)
        top_bar.addWidget(taskspace_title)
        notes = ["Instructions", f"- {chr(945)}, {chr(946)}, {chr(947)} are euler angles in degrees applied in XYZ for the obstacle orientation", 
                 "- Static objects are locked in place and cannot move",
                 "- You must specify your taskspace name on the dashboard to use it after saving it",
                 "- Choose a default object shape (Circle, Square, ...) or specify a path to a STL file",
                 "- There can not be any spaces in your filename or path when specifying a STL file"]
        for i in notes:
            bullet_box = QHBoxLayout()
            bullet_box.addSpacing(20)
            bullet_box.addWidget(QLabel(i))
            top_bar.addLayout(bullet_box)
        main_layout.addLayout(top_bar)
        main_layout.addStretch()
        
        # Bottom part of taskspace creation screen
        taskspace_main = QHBoxLayout()
        GUI_taskspace = QVBoxLayout()
        GUI_taskspace.addWidget(QLabel("Click here to GUI it"))
        
        # Where each new obstacle will be added
        self.manually_add = QVBoxLayout()
        
        # Left Taskspace creation options
        save_obstacles = QPushButton("Save Obstacles to file")
        save_obstacles.setMaximumWidth(300)
        save_obstacles.clicked.connect(self.save_custom_obstacles_taskspace)
        obstacle_save_name_box = QHBoxLayout()
        obstacle_save_name_box.addWidget(QLabel("Taskspace Name: "))
        obstacle_save_name = QLineEdit()
        obstacle_save_name.setPlaceholderText(self.confDict["motionplan_name"])
        obstacle_save_name.setMaximumWidth(250)
        obstacle_save_name.textChanged.connect(self.change_custom_taskspace_file)
        obstacle_save_name_box.addWidget(obstacle_save_name)
        obstacle_save_name_box.addStretch()
        
        
        self.manually_add.addWidget(save_obstacles)
        self.manually_add.addLayout(obstacle_save_name_box)
        add_obstacle = QPushButton("Add object +")
        add_obstacle.setMaximumWidth(300)
        add_obstacle.clicked.connect(self.add_obstacle_button)
        self.manually_add.addWidget(add_obstacle)
        

        
        taskspace_main.addLayout(self.manually_add)
        taskspace_main.addStretch()
        taskspace_main.addLayout(GUI_taskspace)
        main_layout.addLayout(taskspace_main)
        
        taskspace_widget.setLayout(main_layout)
    
    def setup_motionplan(self, motion_widget: QWidget):
        # Entire page layout
        main_layout = QVBoxLayout()
        # Top bar & title layout
        top_bar = QVBoxLayout()
        taskspace_title = QLabel("Create your taskspace here")
        taskspace_title.setFont(self.title_font)
        top_bar.addWidget(taskspace_title)
        notes = ["Instructions", f" - On the left you can manually define a motion plane", 
                 "- This motion plan currently only suppose single segment torque motion",
                 "- Each \'point\' represents a moment in time when the robot will shift control from the previous point to the current one",
                 "- Choose a default object shape (Circle, Square, ...) or specify a path to a STL file",
                 "- There can not be any spaces in your filename or path when specifying a STL file"]
        for i in notes:
            bullet_box = QHBoxLayout()
            bullet_box.addSpacing(20)
            bullet_box.addWidget(QLabel(i))
            top_bar.addLayout(bullet_box)
        main_layout.addLayout(top_bar)
        main_layout.addStretch()
        
        # Bottom part of Motion plan creation screen
        motion_plan_main = QHBoxLayout()
        self.manually_add_plan = QVBoxLayout()
        Automatic_plan = QVBoxLayout()
        Automatic_plan.addWidget(QLabel("Click here to  automatically create a motion plan with XYZ"))
        
        # Where each new point will be added
        save_plan = QPushButton("Save motion plan to file")
        save_plan.setMaximumWidth(300)
        save_plan.clicked.connect(self.save_manual_motionplan)
        plan_name_box = QHBoxLayout()
        plan_name_box.addWidget(QLabel("Motion Plan Name: "))
        plan_name = QLineEdit()
        plan_name.setPlaceholderText("Plan_1.csv")
        plan_name.setMaximumWidth(250)
        plan_name.textChanged.connect(self.change_manual_motionplan_file)
        plan_name_box.addWidget(plan_name)
        plan_name_box.addStretch()
        
        
        self.manually_add_plan.addWidget(save_plan)
        self.manually_add_plan.addLayout(plan_name_box)
        
        add_point = QPushButton("Add point")
        add_point.setMaximumWidth(300)
        add_point.clicked.connect(self.add_manual_plan_point)
        self.manually_add_plan.addWidget(add_point)

        
         

        motion_plan_main.addLayout(self.manually_add_plan)
        motion_plan_main.addLayout(Automatic_plan)
        main_layout.addLayout(motion_plan_main)
            
        motion_widget.setLayout(main_layout)
    
    def setup_help(self, help_widget: QWidget):
        main_layout = QVBoxLayout()
        top_bar = QHBoxLayout()
        
    
    def setup_robot_setup(self, setup_widget: QWidget):
        main_layout = QVBoxLayout()
        top_bar = QHBoxLayout()
        robot_setup_title = QLabel("More advanced robot settings (optional)")
        robot_setup_title.setFont(self.title_font)
        top_bar.addWidget(robot_setup_title)
        main_layout.addLayout(top_bar)
        
        main_setup = QHBoxLayout()
        
        
        thickness_box = QHBoxLayout()
        thickness_box.addWidget(QLabel("Thickness"))
        thickness = QDoubleSpinBox()
        thickness.setMinimum(0.01)
        thickness.setSingleStep(0.005)
        thickness.textChanged.connect(lambda w: self.advancedSettings.update([("thickness", w)]))
        thickness_box.addWidget(thickness)
        main_setup.addLayout(thickness_box)
        
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
        
        # Taskspace name
        taskspace_settings_box = QHBoxLayout()
        taskspace_settings_box.addWidget(QLabel("Taskspace filename: "))
        taskspace_name_line = QLineEdit("taskspace.conf")
        taskspace_name_line.textEdited.connect(self.change_setting_taskspace_name)
        taskspace_settings_box.addWidget(taskspace_name_line)
        self.left.addLayout(taskspace_settings_box)
        
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

    def show_alert(self, message, title="Alert!", level="Information"):
        
        if level == "Question":
            icon = QMessageBox.Question
        elif level == "Information":
            icon = QMessageBox.Information
        elif level == "Warning":
            icon = QMessageBox.Warning
        elif level == "Critical":
            icon = QMessageBox.Critical
        
        msg = QMessageBox()
        msg.setIcon(icon)
        msg.setText(message)
        msg.setWindowTitle(title)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()


app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec()



#simulate("./utils/simulationConf.json")