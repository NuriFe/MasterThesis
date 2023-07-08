import xml.etree.ElementTree as ET

# Create the root element
root = ET.Element("OpenSimDocument", Version="20302")

# Create the ControlSet element
control_set = ET.SubElement(root, "CMC_TaskSet", name="Control Set")

# Create the objects element and its child elements
objects = ET.SubElement(control_set, "objects")

# List of variable names

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
    control_linear = ET.SubElement(objects, "CMC_Joint", name=variable)
    ET.SubElement(control_linear, "on").text = " true "
    ET.SubElement(control_linear, "weight").text = "      1.00000000  "
    ET.SubElement(control_linear, "default_max").text = "       1.00000000 "
    ET.SubElement(control_linear, "active").text = " true false false  "
    ET.SubElement(control_linear, "kp").text = "     100.00000000 "
    ET.SubElement(control_linear, "kv").text = "      20.00000000 "
    ET.SubElement(control_linear, "ka").text = "       1.00000000 "
    ET.SubElement(control_linear, "coordinate").text = variable
    ET.SubElement(control_linear, "limit").text = "       0.00000000 "


# Create the XML tree
tree = ET.ElementTree(root)
ET.indent(tree, '  ')

# Write the XML tree to a file
tree.write("DAS_CMC_Tasks.xml", encoding="utf-8", xml_declaration=True)
