import argparse
import xml.etree.ElementTree as ET


def set_property(xml_root, name, value):
    # changing the field {emission, ambient}
    for elem in xml_root.iter():
        if name in elem.tag:
            color = elem.getchildren()[0]
    if color is None:
        raise IOError("Not found name {}".format(name))
    assert value >= 0, "invalid {} light".format(name)
    assert value <= 1, "invalid {} light".format(name)
    color.text ='{0} {0} {0} 1'.format(value)
    return xml_root

def main(args):

    tree = ET.parse(args.xml_file)

    ET.register_namespace('', "http://www.collada.org/2005/11/COLLADASchema")
    xml_root = tree.getroot()

    if args.ambient:
        set_property(xml_root, "ambient", args.ambient)
    if args.emission:
        set_property(xml_root, "emission", args.emission)

    tree.write(args.xml_file, encoding="utf-8", xml_declaration=True)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Select gate illumination properties.')
    parser.add_argument('-ambient', type=float, required=False)
    parser.add_argument('-xml_file', type=str, default='gate.dae')
    parser.add_argument('-emission', type=float, required=False)
    args = parser.parse_args()
    main(args)
