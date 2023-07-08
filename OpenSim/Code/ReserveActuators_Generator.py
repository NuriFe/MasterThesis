import xml.etree.ElementTree as ET

# Create the root element
root = ET.Element("OpenSimDocument", Version="20302")

# Create the ForceSet element
force_set = ET.SubElement(root, "ForceSet", name="arm26")

# Create the defaults element
defaults = ET.SubElement(force_set, "defaults")

# Create the objects element
objects = ET.SubElement(force_set, "objects")

variables = [
    "SC_y",
    "SC_z",
    "SC_x",
    "AC_y",
    "AC_z",
    "AC_x",
    "GH_y",
    "GH_z",
    "GH_yy",
    "EL_x",
    "PS_y"
]
# Create ControlLinear elements for each variable
for variable in variables:
    # Create the  CoordinateActuator element
    coordinate_actuator1 = ET.SubElement(objects, "CoordinateActuator", name=variable + "_reserve")
    is_disabled1 = ET.SubElement(coordinate_actuator1, "isDisabled")
    is_disabled1.text = "false"
    min_control1 = ET.SubElement(coordinate_actuator1, "min_control")
    min_control1.text = "-infinity"
    max_control1 = ET.SubElement(coordinate_actuator1, "max_control")
    max_control1.text = "infinity"
    coordinate1 = ET.SubElement(coordinate_actuator1, "coordinate")
    coordinate1.text = variable
    optimal_force1 = ET.SubElement(coordinate_actuator1, "optimal_force")
    optimal_force1.text = "1.00000000"

# Create the groups element
groups = ET.SubElement(force_set, "groups")

# Create the XML tree
tree = ET.ElementTree(root)
ET.indent(tree, '  ')

# Write the XML tree to a file
tree.write("DAS_Reserve_Actuators.xml", encoding="UTF-8", xml_declaration=True)
