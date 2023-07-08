import xml.etree.ElementTree as ET

# Create the root element
root = ET.Element("OpenSimDocument", Version="20302")

# Create the ControlSet element
control_set = ET.SubElement(root, "ControlSet", name="Control Set")

# Create the defaults element and its child elements
defaults = ET.SubElement(control_set, "defaults")
control_linear = ET.SubElement(defaults, "ControlLinear", name="default")
ET.SubElement(control_linear, "is_model_control").text = " true "
ET.SubElement(control_linear, "extrapolate").text = " true "
ET.SubElement(control_linear, "default_min").text = "       0.02000000 "
ET.SubElement(control_linear, "default_max").text = "       1.00000000 "
ET.SubElement(control_linear, "filter_on").text = " false "
ET.SubElement(control_linear, "use_steps").text = " true "
ET.SubElement(control_linear, "x_nodes")
ET.SubElement(control_linear, "min_nodes")
ET.SubElement(control_linear, "max_nodes")
ET.SubElement(control_linear, "kp").text = "     100.00000000 "
ET.SubElement(control_linear, "kv").text = "      20.00000000 "

# Create the objects element and its child elements
objects = ET.SubElement(control_set, "objects")

# List of variable names
variables = [
    "trapscap1.excitation",
    "trapscap2.excitation",
    "trapscap3.excitation",
    "trapscap4.excitation",
    "trapscap5.excitation",
    "trapscap6.excitation",
    "trapscap7.excitation",
    "trapscap8.excitation",
    "trapscap9.excitation",
    "trapscap10.excitation",
    "trapscap11.excitation",
    "trapclav1.excitation",
    "trapclav2.excitation",
    "levscap1.excitation",
    "levscap2.excitation",
    "pectmin1.excitation",
    "pectmin2.excitation",
    "pectmin3.excitation",
    "pectmin4.excitation",
    "rhomboid1.excitation",
    "rhomboid2.excitation",
    "rhomboid3.excitation",
    "rhomboid4.excitation",
    "rhomboid5.excitation",
    "serrant1.excitation",
    "serrant2.excitation",
    "serrant3.excitation",
    "serrant4.excitation",
    "serrant5.excitation",
    "serrant6.excitation",
    "serrant7.excitation",
    "serrant8.excitation",
    "serrant9.excitation",
    "serrant10.excitation",
    "serrant11.excitation",
    "serrant12.excitation",
    "deltscap1.excitation",
    "deltscap2.excitation",
    "deltscap3.excitation",
    "deltscap4.excitation",
    "deltscap5.excitation",
    "deltscap6.excitation",
    "deltscap7.excitation",
    "deltscap8.excitation",
    "deltscap9.excitation",
    "deltscap10.excitation",
    "deltscap11.excitation",
    "deltclav1.excitation",
    "deltclav2.excitation",
    "deltclav3.excitation",
    "deltclav4.excitation",
    "coracobr1.excitation",
    "coracobr2.excitation",
    "coracobr3.excitation",
    "infra1.excitation",
    "infra2.excitation",
    "infra3.excitation",
    "infra4.excitation",
    "infra5.excitation",
    "infra6.excitation",
    "termin1.excitation",
    "termin2.excitation",
    "termin3.excitation",
    "termaj1.excitation",
    "termaj2.excitation",
    "termaj3.excitation",
    "termaj4.excitation",
    "supra1.excitation",
    "supra2.excitation",
    "supra3.excitation",
    "supra4.excitation",
    "subscap1.excitation",
    "subscap2.excitation",
    "subscap3.excitation",
    "subscap4.excitation",
    "subscap5.excitation",
    "subscap6.excitation",
    "subscap7.excitation",
    "subscap8.excitation",
    "subscap9.excitation",
    "subscap10.excitation",
    "subscap11.excitation",
    "bicl.excitation",
    "bicb1.excitation",
    "bicb2.excitation",
    "triclong1.excitation",
    "triclong2.excitation",
    "triclong3.excitation",
    "triclong4.excitation",
    "latdorsi1.excitation",
    "latdorsi2.excitation",
    "latdorsi3.excitation",
    "latdorsi4.excitation",
    "latdorsi5.excitation",
    "latdorsi6.excitation",
    "pectmajt1.excitation",
    "pectmajt2.excitation",
    "pectmajt3.excitation",
    "pectmajt4.excitation",
    "pectmajt5.excitation",
    "pectmajt6.excitation",
    "pectmajc1.excitation",
    "pectmajc2.excitation",
    "tricmed1.excitation",
    "tricmed2.excitation",
    "tricmed3.excitation",
    "tricmed4.excitation",
    "tricmed5.excitation",
    "brachialis1.excitation",
    "brachialis2.excitation",
    "brachialis3.excitation",
    "brachialis4.excitation",
    "brachialis5.excitation",
    "brachialis6.excitation",
    "brachialis7.excitation",
    "brachiorad1.excitation",
    "brachiorad2.excitation",
    "brachiorad3.excitation",
    "pronteres1.excitation",
    "pronteres2.excitation",
    "supinator1.excitation",
    "supinator2.excitation",
    "supinator3.excitation",
    "supinator4.excitation",
    "supinator5.excitation",
    "pronquad1.excitation",
    "pronquad2.excitation",
    "pronquad3.excitation",
    "triclat1.excitation",
    "triclat2.excitation",
    "triclat3.excitation",
    "triclat4.excitation",
    "triclat5.excitation",
    "anconeus1.excitation",
    "anconeus2.excitation",
    "anconeus3.excitation",
    "anconeus4.excitation",
    "anconeus5.excitation"
]


# Create ControlLinear elements for each variable
for variable in variables:
    control_linear = ET.SubElement(objects, "ControlLinear", name=variable)
    ET.SubElement(control_linear, "is_model_control").text = " true "
    ET.SubElement(control_linear, "extrapolate").text = " true "
    ET.SubElement(control_linear, "default_min").text = "       0.02000000 "
    ET.SubElement(control_linear, "default_max").text = "       1.00000000 "
    ET.SubElement(control_linear, "filter_on").text = " false "
    ET.SubElement(control_linear, "use_steps").text = " true "
    ET.SubElement(control_linear, "x_nodes")
    ET.SubElement(control_linear, "min_nodes")
    ET.SubElement(control_linear, "max_nodes")
    ET.SubElement(control_linear, "kp").text = "     100.00000000 "
    ET.SubElement(control_linear, "kv").text = "      20.00000000 "

# Create the XML tree
tree = ET.ElementTree(root)
ET.indent(tree, '  ')

# Write the XML tree to a file
tree.write("DAS_CMC_ControlConstraints_2.xml", encoding="utf-8", xml_declaration=True)
