#!/usr/bin/env python

import argparse
import os
from time import sleep
from bluetool import Bluetooth

# Terminal bold colors
BBLACK = '\033[1;30m'       # Black
BRED = '\033[1;31m'         # Red
BGREEN = '\033[1;32m'       # Green
BYELLOW = '\033[1;33m'      # Yellow
BBLUE = '\033[1;34m'        # Blue
BPURPLE = '\033[1;35m'      # Purple
BCYAN = '\033[1;36m'        # Cyan
BWHITE = '\033[1;37m'       # White

# Terminal reset color
COLOR_OFF = '\033[0m'       # Text Reset

# Time periods
TIME_PERIOD = 2.0
SPEAKER_DELAY = 5.0
MAX_TIME_ATTEMPTS = 5.0


def pair_trust(bluetooth, mac):
    print("Pairing with {}...".format(mac))
    bluetooth.trust(mac)
    bluetooth.start_pairing(mac)
    success = bluetooth.get_device_property(mac, "Paired")

    return success


def mac_pair(bluetooth, mac):
    devices = bluetooth.get_devices_to_pair()
    for dev in devices:
        if dev['mac_address'] == mac:
            success = pair_trust(bluetooth, mac)
            return success, dev

    print((BRED + "Couldn't find device {}" + COLOR_OFF).format(mac))
    return False, None


def name_pair(bluetooth, name_pattern):
    devices = bluetooth.get_devices_to_pair()
    for dev in devices:
        if name_pattern in dev['name']:
            success = pair_trust(bluetooth, dev['mac_address'])
            return success, dev

    print((BRED + "Couldn't find device {}" + COLOR_OFF).format(name_pattern))
    return False, None


def interactive_pair(bluetooth):
    print(BWHITE + "Devices availables to pair:" + COLOR_OFF)
    devices = bluetooth.get_devices_to_pair()
    index = 1
    for dev in devices:
        dev['index'] = index
        index += 1
        print(dev)

    user_index = raw_input(
        "Please enter the index number of the device you want to pair:\n")
    try:
        index = int(user_index)
    except ValueError:
        print((BRED + "'{}' in not a number, please enter a valid integer index" +
              COLOR_OFF).format(user_index))
        return False, None

    for dev in devices:
        if dev['index'] == index:
            success = pair_trust(bluetooth, dev['mac_address'])
            return success, dev

    print((BRED + "Couldn't find device with index: {}" + COLOR_OFF).format(index))
    return False, None


def get_device_status(bluetooth, dev):
    dev['Paired'] = bluetooth.get_device_property(dev['mac_address'], "Paired")
    dev['Trusted'] = bluetooth.get_device_property(
        dev['mac_address'], "Trusted")
    dev['Connected'] = bluetooth.get_device_property(
        dev['mac_address'], "Connected")
    print("Paired: {} - Trusted: {} - Connected: {}".format(
        dev['Paired'], dev['Trusted'], dev['Connected']))


def remove_paired_devices(bluetooth, name_pattern=""):
    devices = bluetooth.get_paired_devices()
    if(name_pattern):
        devices = filter(lambda x: name_pattern in x['name'], devices)
    for dev in devices:
        print(
            (BRED + "{} ({})" + COLOR_OFF).format(dev['mac_address'], dev['name']))
        bluetooth.remove(dev['mac_address'])


def read_assigned_mac(path):
    try:
        mac_file = open(path, 'r')
        mac = mac_file.readline()
        mac_file.close()
        return mac.strip()
    except IOError:
        print((BRED + "No local mac file: {}" + COLOR_OFF).format(path))


def write_selected_mac(path, mac):
    try:
        mac_file = open(path, 'w')
        mac_file.write("{}\n".format(mac))
        mac_file.close()
        print((BWHITE + "Mac {} " + BGREEN + "updated successfully" +
               COLOR_OFF).format(mac))
    except IOError:
        print((BRED + "Couldn't write to mac file: {}" + COLOR_OFF).format(path))

########################### MAIN ###########################


def main(args):
    # Defining device specific properties
    if(args.speaker):
        file_name = "speaker_mac.txt"
        name_pattern = "JBL"
    else:
        file_name = "joy_mac.txt"
        name_pattern = "Xbox Wireless Controller"

    # Setup of device specific values
    mac_path = (os.path.expanduser("~") + "/braintemp/{}").format(file_name)
    bluetooth = Bluetooth()

########################### Removing ###########################
    if(args.remove):
        print(("Removing devices " + BRED + "{} " +
              COLOR_OFF + "...").format(name_pattern))
        remove_paired_devices(bluetooth, name_pattern)
        write_selected_mac(mac_path, '')
        if (args.exit_on_remove):
            return

    if(args.clean):
        print(BYELLOW + "Removing all paired devices ..." + COLOR_OFF)
        remove_paired_devices(bluetooth)
        return

########################### Pairing ###########################

    print((BWHITE + "Scanning for {} secs ...." + COLOR_OFF).format(args.timeout))
    bluetooth.scan(args.timeout)

    if(args.interactive):
        success, device = interactive_pair(bluetooth)
    elif(args.mac):
        success, device = mac_pair(bluetooth, args.mac)
    else:
        success, device = name_pair(bluetooth, name_pattern)
        if(device is not None):
            write_selected_mac(mac_path, device['mac_address'])

    if(device is not None):
        print("Getting {} ({}) info... ".format(
            device['name'], device['mac_address']))
        get_device_status(bluetooth, device)
        pairing_time = attempts_time = 0
        while(not (device['Paired'] and device['Connected'])):
            if (not device['Paired'] and attempts_time > MAX_TIME_ATTEMPTS):
                pair_trust(bluetooth, device['mac_address'])
                attempts_time = 0

            get_device_status(bluetooth, device)
            pairing_time += TIME_PERIOD
            attempts_time += TIME_PERIOD
            sleep(TIME_PERIOD)

            if (pairing_time >= 30):
                break

        # Defining device specific workflow
        if(args.speaker):
            sleep(SPEAKER_DELAY)
            get_device_status(bluetooth, device)
            bluetooth.connect(device['mac_address'])
            get_device_status(bluetooth, device)
            if(device['Paired'] and device['Connected']):
                print((BWHITE + "Device {} ({}) " + BGREEN + "paired successfully" +
                       COLOR_OFF).format(device['name'], device['mac_address']))
            else:
                print((BWHITE + "Couldn't pair Device " + BRED + "{} ({})" +
                      COLOR_OFF).format(device['name'], device['mac_address']))
                bluetooth.remove(device['mac_address'])
                write_selected_mac(mac_path, '')
        else:  # Xbox controller
            if(device['Trusted'] and device['Connected']):
                print((BWHITE + "Device {} ({}) " + BGREEN + "paired successfully" +
                       COLOR_OFF).format(device['name'], device['mac_address']))
            else:
                print((BWHITE + "Couldn't pair Device " + BRED + "{} ({})" +
                      COLOR_OFF).format(device['name'], device['mac_address']))
                bluetooth.remove(device['mac_address'])
                write_selected_mac(mac_path, '')


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument("-s", "--speaker", help="Select speaker as device pairing (default: joystick)",
                            action="store_true")
    arg_parser.add_argument("-i", "--interactive", help="Interactive pairing mode",
                            action="store_true")
    arg_parser.add_argument("-r", "--remove", help="Remove paired devices (matching to device type)",
                            action="store_true")
    arg_parser.add_argument("-e", "--exit_on_remove", help="Exit once all paired devices are removed (matching to device type)",
                            action="store_true")
    arg_parser.add_argument("-c", "--clean", help="Remove all previous paired devices",
                            action="store_true")
    arg_parser.add_argument(
        "-t", "--timeout", help="Scanning time to wait for devices", type=float, default=15)
    arg_parser.add_argument(
        "-m", "--mac", help="Scanning time to wait for devices", type=str, default="")

    args = arg_parser.parse_args()
    main(args)
