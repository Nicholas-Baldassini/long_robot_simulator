import xml.etree.ElementTree as ET

def create_MJCF(num, len, extra=None, movement=False, destination='./MJCFS/new_cont.xml', colour_scheme=True, gravity=True):
    # Config params
    filename = destination
    joints_num = num
    thickness = 0.035
    length = len
    joint_range = "-40.5 40.5"

    # Root/subroot elements
    mujoco = ET.Element('mujoco')
    default = ET.SubElement(mujoco, "default")
    worldbody = ET.SubElement(mujoco, "worldbody")
    actuators = ET.SubElement(mujoco, "actuator")
    asset = ET.SubElement(mujoco, "asset")
    # ----------------------------

    # Default stuff and classes
    position_class = ET.SubElement(default, "default", attrib={'class': 'position'})
    ET.SubElement(position_class, "position",  attrib={'ctrllimited':'true','forcelimited':'false' })

    geom_attribs = {'conaffinity':'0','contype':'1', "friction": "0 0.0 0.0", "margin": "0.001", "rgba": "0.75 0.6 0.5 1", "solimp": "0.95 0.95 0.01", "solref": "0.008 1"}
    geom0_class = ET.SubElement(default, "default", attrib={'class': 'geom0'})
    ET.SubElement(geom0_class, "geom",  attrib=geom_attribs)

    link_class = ET.SubElement(default, "default", attrib={'class': 'link'})
    ET.SubElement(link_class, "joint",  attrib= {'armature':'0.1','damping':'5', "limited": "true", "stiffness": "0"})
    # ----------------------------

    # Lower gravity
    if gravity:
        ET.SubElement(mujoco, "option", attrib={"gravity": "0 0 -1.8"})
    # ----------------------------

    # Main world body
    plane_attrib = {"type": "plane", "size": "100 100 0.1", "friction": "0 0 0"}
    if colour_scheme == "Cinematic":
        ET.SubElement(asset, "texture", attrib={"name": "grid", "type": "2d", "builtin": "checker", "width": "512", "height": "512", "rgb1": ".1 .2 .3", "rgb2": ".2 .3 .4"})
        ET.SubElement(asset, "material", attrib={"name": "grid", "texture": "grid", "texrepeat": "1 1", "texuniform": "true", "reflectance": ".2"})
        plane_attrib["material"] = "grid"
    else:
        plane_attrib["rgba"] = "0.984 0.984 0.984 1"
    ET.SubElement(worldbody, "light", attrib={"pos": "0 0 10", "dir": "0 0 -1", "castshadow": "false"})
    ET.SubElement(worldbody, "geom", attrib=plane_attrib) #"material" : "plane"
    # ----------------------------


    # Add obstacles
    if extra:
       # print(extra, type(extra))
        assert type(extra) is list
        for i in extra:
            worldbody.insert(2, i)



    # Add base sphere and first link
    pos_approx = joints_num * len 
    #print(position_approx)
    base_sphere = ET.SubElement(worldbody, "body", attrib={"name": "base", "pos": f"-{pos_approx} 0 0.05"})
    #ET.SubElement(base_sphere, "joint", attrib={"type": "free", "damping": "0.9", "stiffness": "1.2"})
    ET.SubElement(base_sphere, "geom", attrib={"size": "0.05", "type": "sphere"})
    if movement:
        ET.SubElement(base_sphere, "joint", attrib={"axis": "1 0 0", "name": "sphere_velocity", "pos": "0 0 0", "range": "-200 200", "type": "slide"})
        ET.SubElement(actuators, "velocity", attrib={"ctrlrange": "-4 4", "joint": "sphere_velocity", "name": f"sphere_velocity_act"})
        #ET.SubElement(base_sphere, "inertial", attrib={"pos": "0 0 10", "mass": "1000000"})
    # ----------------------------

    # Add links and joints
    colours = ["0.274 0.274 0.274 1", "0.196 0.196 0.196 1"]
    if colour_scheme == "Cinematic":
        colours = ["0.552 0.549 0.960 0.8", "0.01 0.517 1 0.9"]
    joint_names = []
    base_joint_name = "base_joint"
    joint_names.append(base_joint_name)
    base_link = ET.SubElement(base_sphere, "body", attrib={"name": "base_link", "pos": "0.05 0 0"})
    ET.SubElement(base_link, "geom", attrib={"class": "geom0", "fromto": f"0 0 0 {length} 0 0", "size": f"{thickness}", "type": "capsule"})
    ET.SubElement(base_link, "joint", attrib={"axis": "0 0 1", "class": "link", "name": base_joint_name, "pos": "0 0 0", "range": joint_range, "type": "hinge"})
    #ET.SubElement(base_link, "inertial", attrib={"pos": "0 0 10", "mass": "1"})

    curr_tip = base_link
    i = 0
    while i < joints_num:
        j_name = f"joint_{i}"
        joint_names.append(j_name)
        curr_tip = ET.SubElement(curr_tip, "body", attrib={"name": f"link_{i}", "pos": f"{length} 0 0"})
        ET.SubElement(curr_tip, "geom", attrib={ "fromto": f"0 0 0 {length} 0 0", "size": "0.035", "type":"capsule", "rgba": f"{colours[i % 2]}", "friction": "0 0 0"})
        ET.SubElement(curr_tip, "joint", attrib={"axis": "0 0 1", "class": "link", "name":j_name, "pos": "0 0 0", "range": joint_range, "type": "hinge", "stiffness": "10"})
        #ET.SubElement(curr_tip, "inertial", attrib={"pos": "0 0 10", "mass": "1"})
        i += 1


    # Add actuators/joint control
    for j in joint_names:
        # Position control
        ET.SubElement(actuators, "motor", attrib={"class": "position", "ctrlrange": joint_range, "joint": j, "name": f"act_{j}"})

    space_comment = ET.Comment('\n\t\t')
    mujoco.insert(1, ET.Comment(' === Physical robot definitions === '))
    mujoco.insert(0, ET.Comment(' === Configuration parameter stuff, default values === '))
    mujoco.insert(4, space_comment)
    worldbody.insert(2, space_comment)
    tree = ET.ElementTree(mujoco)
    ET.indent(tree, space="    ", level=0)
    tree.write(filename)
    
    # fix base, add obstacles
    # 
    
    
def create_obstacles(filename, num, pos=None):
    
    if num:
        assert len(pos) == num
    
    bodies = []
    for i in range(num):
        body = ET.Element("body", attrib={"name": f"obstacle_{i}", "pos": f"{pos[i][0]} {pos[i][1]} {pos[i][2]}"})
        #ET.SubElement(body, "joint", attrib={"name": f"obstacle_{i}_joint", "type": "free", "damping": "0.001"})
        ET.SubElement(body, "geom", attrib={"name": f"obstacle_{i}_geom", "size": "0.08", "rgba": "0.98 0.75 1 1", "fromto": "0 0 0 0 0 0.2", "type": "capsule"})
        bodies.append(body)
        
    return bodies


def generate_from_file(filename, colour_scheme=None):
    
    bodies = []
    with open(filename) as f:
        for ind, i in enumerate(f.readlines()):
            i = i.split(" ")
            shape, radius, y, x, static = i[0], i[1], i[2], i[3], i[4][:-1]
            height = 0
            s_colour = "0.411 0.580 0.815 1"
            f_colour = "0.501 0.733 0.568 1"
            if colour_scheme == "Cinematic":
                
                s_colour = "0 0.5 1 1"
                f_colour = "0 0 0 1"
            
            if static == "False":
                
                
                body = ET.Element("body", attrib={"name": f"obstacle_custom_{ind}", "pos": f"{x} {y} {2}"})
                ET.SubElement(body, "joint", attrib={"name": f"obstacle_custom_{ind}_joint", "type": "free", "damping": "0.001"})
                ET.SubElement(body, "geom", attrib={"name": f"obstacle_custom_{ind}_geom", "size": f"{radius}", "rgba": f_colour, "fromto": "0 0 0 0 0 0.01", "type": "capsule"})
            else:
                body = ET.Element("body", attrib={"name": f"obstacle_custom_{ind}", "pos": f"{x} {y} {height}"})
                #ET.SubElement(body, "joint", attrib={"name": f"obstacle_custom_{ind}_joint", "type": "free", "damping": "0.001"})
                ET.SubElement(body, "geom", attrib={"name": f"obstacle_custom_{ind}_geom", "size": f"{radius}", "rgba": s_colour, "fromto": "0 0 0 0 0 0.2", "type": "cylinder"})
            bodies.append(body)
    return bodies






