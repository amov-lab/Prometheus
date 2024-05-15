#!/usr/bin/env python3


import jinja2
import argparse
import os
import shutil
import fnmatch
import numpy as np


def get_file_contents(filepath):
    with open(filepath, 'rb') as f:
        return f.read()

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help="file that the sdf file should be generated from")
    parser.add_argument('env_dir')
    parser.add_argument('--mavlink_tcp_port', default=4560, help="TCP port for PX4 SITL")
    parser.add_argument('--mavlink_udp_port', default=14560, help="Mavlink UDP port for mavlink access")
    parser.add_argument('--serial_enabled', default=0, help="Enable Serial device for HITL")
    parser.add_argument('--serial_device', default="/dev/ttyACM0", help="Serial device for FMU")
    parser.add_argument('--serial_baudrate', default=921600, help="Baudrate of Serial device for FMU")
    parser.add_argument('--hil_mode', default=0, help="Enable HIL mode for HITL simulation")
    parser.add_argument('--output-file', help="sdf output file")
    parser.add_argument('--stdout', action='store_true', default=False, help="dump to stdout instead of file")
    parser.add_argument('--mavlink_id', default=1, help="Mavlink system ID")
    parser.add_argument('--cam_component_id', default=100, help="Mavlink camera component ID")
    parser.add_argument('--gst_udp_port', default=5600, help="Gstreamer UDP port for SITL")
    parser.add_argument('--video_uri', default=5600, help="Mavlink camera URI for SITL")
    parser.add_argument('--mavlink_cam_udp_port', default=14530, help="Mavlink camera UDP port for SITL")
    parser.add_argument('--generate_ros_models', default=False, dest='generate_ros_models', type=str2bool,
                    help="required if generating the agent for usage with ROS nodes, by default false")
    args = parser.parse_args()
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.env_dir))
    template = env.get_template(os.path.relpath(args.filename, args.env_dir))

    # get ROS version, if generate_ros_models is true.
    ros_version = 0
    if args.generate_ros_models:
        ros_version = os.environ.get('ROS_VERSION')

    # create dictionary with useful modules etc.
    try:
        import rospkg
        rospack = rospkg.RosPack()
    except ImportError:
        pass
        rospack = None

    d = {'np': np, 'rospack': rospack, \
         'mavlink_tcp_port': args.mavlink_tcp_port, \
         'mavlink_udp_port': args.mavlink_udp_port, \
         'serial_enabled': args.serial_enabled, \
         'serial_device': args.serial_device, \
         'serial_baudrate': args.serial_baudrate, \
         'mavlink_id': args.mavlink_id, \
         'cam_component_id': args.cam_component_id, \
         'gst_udp_port': args.gst_udp_port, \
         'video_uri': args.video_uri, \
         'mavlink_cam_udp_port': args.mavlink_cam_udp_port, \
         'hil_mode': args.hil_mode, \
         'ros_version': ros_version}

    result = template.render(d)

    if args.stdout:
        print(result)

    else:
        if args.output_file:
            filename_out = args.output_file
        else:
            if not args.filename.endswith('.sdf.jinja'):
                raise Exception("ERROR: Output file can only be determined automatically for " + \
                                "input files with the .sdf.jinja extension")
            filename_out = args.filename.replace('.sdf.jinja', '.sdf')
            assert filename_out != args.filename, "Not allowed to overwrite template"

        # Overwrite protection mechanism: after generation, the file will be copied to a "last_generated" file.
        # In the next run, we can check whether the target file is still unmodified.
        filename_out_last_generated = filename_out + '.last_generated'

        if os.path.exists(filename_out) and os.path.exists(filename_out_last_generated):
            # Check whether the target file is still unmodified.
            if get_file_contents(filename_out).strip() != get_file_contents(filename_out_last_generated).strip():
                raise Exception("ERROR: generation would overwrite changes to `{}`. ".format(filename_out) + \
                                "Changes should only be made to the template file `{}`. ".format(args.filename) + \
                                "Remove `{}` ".format(os.path.basename(filename_out)) + \
                                "(after extracting your changes) to disable this overwrite protection.")

        with open(filename_out, 'w') as f_out:
            print(('{:s} -> {:s}'.format(args.filename, filename_out)))
            f_out.write(result)

        # Copy the contents to a "last_generated" file for overwrite protection check next time.
        shutil.copy(filename_out, filename_out_last_generated)
