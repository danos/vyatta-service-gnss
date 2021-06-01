#!/usr/bin/python3

# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
Support for the ublox GNSS device in the S9500 30XS platform.
"""

import argparse
import array
import datetime
import json
import usb.core
import usb.util

from vyatta.gnss import GNSS
from vyatta.platform.detect import PlatformError, detect

import timing_utility

from CPLD_utility import CPLDUtility
from const.const import Led
from protocol.lpc import LPC
from protocol.lpc import LPCDevType
from timing.gpsusb import GPSUSB


class UbloxGNSS:
    """
    A helper class that registers the class that implements GNSS plugin.
    """
    def __init__(self):
        pass

    def probe(self, register):
        """
        If we are running on the S9500-30XS, register the plugin.
        """
        try:
            platform = detect()
            if platform.get_platform_string() == 'ufi.s9500-30xs':
                instance = _UbloxGNSS()
                register(instance)
                print('Adding UbloxGNSS instance', instance.get_instance())
        except PlatformError:
            pass


def gnss_dpll_state():
    """
    Check the state of the 1PPS and 10MHz timing signals sent
    to the IDT clock management chip from the ublox GNSS.

    If neither is valid, then the GNSS is free running (or
    no signal). If the only 10Mhz is valid, the GNSS is acquiring
    a lock. When both are valid, then the GNSS has a valid time fix.
    """
    timing_util = timing_utility.TimingUtility()

    one_pps = timing_util.get_dpll_input_status('GPS-1PPS')
    ten_mhz = timing_util.get_dpll_input_status('GPS-10MHz')
    return one_pps['valid_dpll1'], ten_mhz['valid_dpll2']


def gnss_antenna_status():
    """
    Fetch the antenna status from the CPLD register 0x1c.
    Two bits are used to indicate the short or open state.

    D[0] GNSS antenna shorted status
         0: GNSS antenna is shorted circuited
         1: GNSS antenna is not shorted circuited
    D[1] GNSS antenna open status
         0: GNSS antenna is open circuit
         1: GNSS antenna is not open circuit
    D[2..7] Reserved
    """
    lpc = LPC()
    GNSS_STATUS_REGISTER = 0x1c

    value = lpc.regGet(LPCDevType.CPLD_ON_MAIN_BOARD, GNSS_STATUS_REGISTER)
    is_shorted = value & 0x1
    is_open = value & 0x2
    return is_shorted, is_open


def usb_write(usb_dev, cmd):
    """
    USB write to the Ublox GNSS device.
    """
    cfg = usb_dev.usb_dev.get_active_configuration()
    intf = cfg[(1, 0)]

    endpoint = usb.util.find_descriptor(
               intf,
               # match the first OUT endpoint
               custom_match=lambda e:
               usb.util.endpoint_direction(e.bEndpointAddress) ==
               usb.util.ENDPOINT_OUT)
    if endpoint is None:
        raise ValueError('EndpointAddress of USB device not found')

    endpoint.write(cmd)


class _UbloxGNSS(GNSS):
    """
    ublox GNSS device support for the S9500 30XS platform.
    """
    MAX_RECORDS = 20

    def __init__(self):
        print('Opening the GPS...')
        usb_dev = GPSUSB()

        print('Configuring the GPS...')
        usb_dev.enable()
        usb_dev.configureUartTod()

        # Enable GSV sentences
        cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                0xf0, 0x03, 0x00, 0x00, 0x00, 0x01,
                                0x00, 0x01, 0x04, 0x3c])
        usb_write(usb_dev, cmd)

        # Enable RMC sentences
        cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                0xf0, 0x04, 0x01, 0x00, 0x00, 0x01,
                                0x01, 0x01, 0x07, 0x4b])
        usb_write(usb_dev, cmd)

        self.have_time = False
        self.gpstime = 0.0
        self.have_position = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.have_satellites = False
        self.satellites = []
        self.enabled = True

        self.update_hardware_status()

    def start(self):
        """
        Start the ublox GNSS device.
        """

        # UBX-CFG-RST: resetMode = 0x09 -- controller start
        cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                0x00, 0x00, 0x09, 0x00, 0x17, 0x76])
        usb_dev = GPSUSB()
        usb_write(usb_dev, cmd)
        self.enabled = True

        return True

    def stop(self):
        """
        Stop the ublox GNSS device.
        """

        # UBX-CFG-RST: resetMode = 0x08 -- controller stop
        cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                0x00, 0x00, 0x08, 0x00, 0x16, 0x74])
        usb_dev = GPSUSB()
        usb_write(usb_dev, cmd)
        self.enabled = False

        return True

    def get_status(self):
        """
        Get the status of ublox GNSS device.
        """
        is_shorted, is_open = gnss_antenna_status()
        state_table = {
            (0, 0): "short",
            (0, 1): "OK",
            (1, 0): "unknown",
            (1, 1): "open",
        }
        antenna_status = state_table[is_open, is_shorted]

        tracking_status = 'unknown'
        (one_pps, ten_mhz) = gnss_dpll_state()
        if one_pps and ten_mhz:
            tracking_status = 'tracking'
        elif ten_mhz:
            tracking_status = 'acquiring'

        status = {'instance-number': self.get_instance(),
                  'antenna-status': antenna_status,
                  'enabled': self.enabled,
                  'system': 'gps-system',
                  'tracking-status': tracking_status}

        # If the GNSS is stopped, we won't be able to
        # fetch any additional information.
        if self.enabled:
            self.fetch_gnss_data()

            if self.have_time:
                status['time'] = self.gpstime

            if self.have_position:
                status['latitude'] = str(self.latitude)
                status['longitude'] = str(self.longitude)

            if self.have_satellites:
                status['satellites-in-view'] = self.satellites

        return status

    def decode_gsv(self, fields):
        """
        Decode a GSV NMEA sentence

        $GPGSV,3,1,12,01,22,253,23,07,04,268,19,08,80,281,12,10,51,072,*73
        $GPGSV,3,2,12,14,08,327,,16,16,176,12,21,48,259,23,22,14,203,20*75
        $GPGSV,3,3,12,23,23,046,,27,61,125,10,30,09,302,14,32,11,116,*74
        """
        num_sentences = int(fields[1])
        ith_sentence = int(fields[2])

        if ith_sentence == 1:
            self.satellites = []

        for index in range(4, len(fields), 4):
            satellite = {}
            satellite['instance-number'] = len(self.satellites)
            satellite['PRN'] = fields[index]
            satellite['elevation'] = str(fields[index + 1])
            satellite['azimuth'] = str(fields[index + 2])
            snr = fields[index + 3]
            if snr != '':
                satellite['SNR'] = float(snr)
            self.satellites.append(satellite)

        if num_sentences == ith_sentence:
            self.have_satellites = True

    def decode_rmc(self, fields):
        """
        Decode a RMC NMEA sentence

        $GPRMC,144340.00,A,5127.30667,N,00058.67664,W,0.047,,020621,,,A*6D
        """
        degrees = float(fields[3][0:2])
        minutes = float(fields[3][2:])
        latitude = degrees + minutes / 60.0
        if fields[4] == 'S':
            latitude = -latitude
        degrees = float(fields[5][0:3])
        minutes = float(fields[5][3:])
        longitude = degrees + minutes / 60.0
        if fields[6] == 'W':
            longitude = -longitude
        self.longitude = longitude
        self.latitude = latitude
        self.have_position = True

    def decode_zda(self, fields):
        """
        Decode a ZDA NMEA sentence

        $GPZDA,144340.00,02,06,2021,00,00*65
        """
        hour = int(fields[1][0:2])
        mins = int(fields[1][2:4])
        secs = int(fields[1][4:6])
        day = int(fields[2])
        month = int(fields[3])
        year = int(fields[4])
        utc = datetime.datetime(year, month, day,
                                hour=hour, minute=mins, second=secs)
        self.gpstime = int(utc.timestamp())
        self.have_time = True

    def decode_sentence(self, sentence):
        """
        Break a NMEA into fields and send to the appropriate parser.
        """
        fields = sentence.rstrip().rpartition('*')[0].split(',')

        if fields[0].find('RMC') > 0:
            self.decode_rmc(fields)

        if fields[0].find('ZDA') > 0:
            self.decode_zda(fields)

        if fields[0].find('GPGSV') > 0:
            self.decode_gsv(fields)

    def fetch_gnss_data(self):
        """
        Poll the GNSS data stream for NMEA sentences.
        """
        POLL_TIMEOUT = 1000     # 1s

        usb_dev = GPSUSB()
        cfg = usb_dev.usb_dev.get_active_configuration()
        intf = cfg[(1, 0)]

        endpoint = usb.util.find_descriptor(
                   intf,
                   # match the first IN endpoint
                   custom_match=lambda e:
                   usb.util.endpoint_direction(e.bEndpointAddress) ==
                   usb.util.ENDPOINT_IN)
        if endpoint is None:
            raise ValueError('EndpointAddress of USB device not found')

        self.have_time = False
        self.have_position = False
        self.have_satellites = False

        records_read = 0

        read = True
        while read:
            try:
                resp = endpoint.read(POLL_TIMEOUT)
                self.decode_sentence(resp.tobytes().decode('utf-8'))

                if self.have_time and \
                   self.have_position and \
                   self.have_satellites:
                    read = False

                records_read += 1
                if records_read > self.MAX_RECORDS:
                    read = False
            except:
                # Continuous read until timeout
                pass

    def update_hardware_status(self):
        """
        Update the GPS status LED.
        """
        is_shorted, is_open = gnss_antenna_status()

        # open=1, shorted=1                         => unconnected
        #
        # open=0, shorted=0                         => malfunction
        #
        # open=0, shorted=1,
        # dpll_state = holdover|phase-lost|free-run => no-signal
        #
        # open=0, shorted=1,
        # dpll_state = pre-locked1|pre-locked2      => acquiring
        #
        # open=0, shorted=1,
        # dpll_state = pre-locked1|pre-locked2      => locked

        if is_open == 1 and is_shorted == 1:
            on_off = Led.STATUS_OFF
            color = Led.COLOR_GREEN
            blink = Led.BLINK_STATUS_SOLID
        elif is_open == 0 and is_shorted == 0:
            on_off = Led.STATUS_ON
            color = Led.COLOR_YELLOW
            blink = Led.BLINK_STATUS_BLINKING
        elif is_open == 0 and is_shorted == 1:
            (one_pps, ten_mhz) = gnss_dpll_state()

            on_off = Led.STATUS_ON
            if one_pps and ten_mhz:
                color = Led.COLOR_GREEN
                blink = Led.BLINK_STATUS_SOLID
            elif ten_mhz:
                color = Led.COLOR_GREEN
                blink = Led.BLINK_STATUS_BLINKING
            else:
                color = Led.COLOR_YELLOW
                blink = Led.BLINK_STATUS_SOLID

        cpld_util = CPLDUtility()
        cpld_util.set_led_control(Led.GPS, on_off, color, blink)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--status", action='store_true',
                       help='Get the state of the ublox GNSS')
    group.add_argument("--start", action='store_true',
                       help='Start the ublox GNSS')
    group.add_argument("--stop", action='store_true',
                       help='Stop the ublox GNSS')
    group.add_argument("--update", action='store_true',
                       help='Update the hardware LEDs')
    args = parser.parse_args()

    gnss = _UbloxGNSS()
    gnss.set_instance(0)

    if args.status:
        print(json.dumps(gnss.get_status()))
    elif args.start:
        gnss.start()
    elif args.stop:
        gnss.stop()
    elif args.update:
        gnss.update_hardware_status()
    else:
        parser.print_help()
