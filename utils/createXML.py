import xml.etree.ElementTree as ET

def create_MJCF(num, len, extra=None, destination='./MJCFS/new_cont.xml'):
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
    # ----------------------------

    # Default stuff and classes
    position_class = ET.SubElement(default, "default", attrib={'class': 'position'})
    pos = ET.SubElement(position_class, "position",  attrib={'ctrllimited':'true','forcelimited':'false' })

    geom_attribs = {'conaffinity':'0','contype':'1', "friction": "1 0.005 0.001", "margin": "0.001", "rgba": "0.75 0.6 0.5 1", "solimp": "0.95 0.95 0.01", "solref": "0.008 1"}
    geom0_class = ET.SubElement(default, "default", attrib={'class': 'geom0'})
    geom0 = ET.SubElement(geom0_class, "geom",  attrib=geom_attribs)

    link_class = ET.SubElement(default, "default", attrib={'class': 'link'})
    link = ET.SubElement(link_class, "joint",  attrib= {'armature':'0.1','damping':'5', "limited": "true", "stiffness": "0"})
    # ----------------------------

    # Lower gravity
    gravity = ET.SubElement(mujoco, "option", attrib={"gravity": "0 0 -1.8"})
    # ----------------------------

    # Main world body
    light = ET.SubElement(worldbody, "light", attrib={"diffuse": "1.0 1.0 1.0", "pos": "0 0 10", "dir": "0 0 -1"})
    surface = ET.SubElement(worldbody, "geom", attrib={"type": "plane", "size": "100 100 0.1", "rgba": "0.46 0.86 1 1"})
    # ----------------------------


    # Add obstacles
    if extra:
       # print(extra, type(extra))
        assert type(extra) is list
        for i in extra:
            worldbody.insert(2, i)



    # Add base sphere and first link
    base_sphere = ET.SubElement(worldbody, "body", attrib={"name": "base", "pos": "0 0 0.04"})
    #ET.SubElement(base_sphere, "joint", attrib={"type": "free", "damping": "0.9", "stiffness": "1.2"})
    ET.SubElement(base_sphere, "geom", attrib={"size": "0.05", "type": "sphere"})
    # ----------------------------

    # Add links and joints
    joint_names = []
    base_joint_name = "base_joint"
    joint_names.append(base_joint_name)
    base_link = ET.SubElement(base_sphere, "body", attrib={"name": "base_link", "pos": "0.05 0 0"})
    ET.SubElement(base_link, "geom", attrib={"class": "geom0", "fromto": f"0 0 0 {length} 0 0", "size": f"{thickness}", "type": "capsule"})
    ET.SubElement(base_link, "joint", attrib={"axis": "0 0 1", "class": "link", "name": base_joint_name, "pos": "0 0 0", "range": joint_range, "type": "hinge"})

    curr_tip = base_link
    i = 0
    while i < joints_num:
        j_name = f"joint_{i}"
        joint_names.append(j_name)
        curr_tip = ET.SubElement(curr_tip, "body", attrib={"name": f"link_{i}", "pos": f"{length} 0 0"})
        ET.SubElement(curr_tip, "geom", attrib={"class": "geom0", "fromto": f"0 0 0 {length} 0 0", "size": "0.035", "type":"capsule"})
        ET.SubElement(curr_tip, "joint", attrib={"axis": "0 0 1", "class": "link", "name":j_name, "pos": "0 0 0", "range": joint_range, "type": "hinge", "stiffness": "10"})
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


def generate_from_file(filename):
    
    bodies = []
    with open(filename) as f:
        for ind, i in enumerate(f.readlines()):
            i = i.split(" ")
            shape, radius, y, x, static = i[0], i[1], i[2], i[3], i[4][:-1]
            height = 0
            
            if static == "False":
                body = ET.Element("body", attrib={"name": f"obstacle_custom_{ind}", "pos": f"{x} {y} {2}"})
                ET.SubElement(body, "joint", attrib={"name": f"obstacle_custom_{ind}_joint", "type": "free", "damping": "0.001"})
                ET.SubElement(body, "geom", attrib={"name": f"obstacle_custom_{ind}_geom", "size": f"{radius}", "rgba": "0 0 0 1", "fromto": "0 0 0 0 0 0.01", "type": "capsule"})
            else:
                body = ET.Element("body", attrib={"name": f"obstacle_custom_{ind}", "pos": f"{x} {y} {height}"})
                #ET.SubElement(body, "joint", attrib={"name": f"obstacle_custom_{ind}_joint", "type": "free", "damping": "0.001"})
                ET.SubElement(body, "geom", attrib={"name": f"obstacle_custom_{ind}_geom", "size": f"{radius}", "rgba": "0.98 0.75 1 1", "fromto": "0 0 0 0 0 0.2", "type": "capsule"})
            bodies.append(body)
    return bodies