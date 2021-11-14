#!/usr/bin/python3

# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
Support for the ublox GNSS device in the S9500 30XS platform.
"""

from enum import Enum, unique

import argparse
import array
import datetime
import json
import math
import os
import time
import syslog
import usb.core
import usb.util
import vci

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


def gnss_antenna_state():
    """
    Fetch the antenna state from the CPLD register 0x1c.
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
    is_shorted = value & 0x1 == 0x1
    is_open = value & 0x2 == 0x2
    return is_shorted, is_open


def gnss_antenna_status(is_shorted, is_open):
    """
    Return a valid antenna-status-enumeration from vyatta-service-gnss-v1
    """
    state_table = {
        (0, 0): "short",
        (0, 1): "unknown",
        (1, 0): "OK",
        (1, 1): "open",
    }
    return state_table[is_shorted, is_open]


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


@unique
class LedState(Enum):
    """
    Possible states for an LED
    """
    UNKNOWN = 0
    OFF = 1
    GREEN = 2
    BLINKING_GREEN = 3
    YELLOW = 4
    BLINKING_YELLOW = 5


class TimingState(Enum):
    """
    Possible states for the system timing core.
    """
    PHASE_LOCKED = 1
    FREQUENCY_LOCKED = 2
    HOLDOVER = 3
    FREE_RUN = 4


def dpll_get_timing_state():
    """
    Check and report the state of the system timing. If either
    DPLL is locked or in a pre-locked state, we report that.
    If we are in any of the other states, we choose Holdover
    or Free Run.

    DPLL1        DPLL2            DPLL1 or DPLL2
    PHASE_LOCK > FREQUENCY_LOCK > HOLDOVER       > FREE_RUN
    """
    def is_locked(dpll):
        if dpll['current'] is not None and \
           dpll['status']['operating_status'] in \
           ('Locked', 'Pre-locked', 'Pre-locked2'):
            return True
        return False

    def is_holdover(dpll):
        if dpll['status']['operating_status'] == 'Holdover':
            return True
        return False

    if not hasattr(dpll_get_timing_state, "previously_locked"):
        dpll_get_timing_state.previously_locked = False

    DPLL1 = 1
    DPLL2 = 2

    timing_util = timing_utility.TimingUtility()
    dpll1 = timing_util.get_dpll_status(DPLL1)
    dpll2 = timing_util.get_dpll_status(DPLL2)

    timing_state = TimingState.FREE_RUN

    if is_locked(dpll1):
        timing_state = TimingState.PHASE_LOCKED
        dpll_get_timing_state.previously_locked = True
    elif is_locked(dpll2):
        timing_state = TimingState.FREQUENCY_LOCKED
        dpll_get_timing_state.previously_locked = True
    elif (is_holdover(dpll1) or is_holdover(dpll2)) and \
            dpll_get_timing_state.previously_locked:
        timing_state = TimingState.HOLDOVER

    return timing_state


def checksum(command):
    """
    Calculate the UBX checksum for a given command
    """
    ck_a = 0
    ck_b = 0
    for b in command[2:]:
        ck_a += b
        ck_b += ck_a
    return [ ck_a % 256, ck_b % 256  ]


def platform_delay():
    """
    It is typical for a platform to have some internal delay
    that is inherent to the hardware.
    """
    if os.path.exists('/run/vyatta/platform/ufi.s9500-30xs'):
        return 50

    return 0


def ubx_cfg_tp5(usb_dev, delay):
    """
    Set the antenna delay using the time pulse parameters.
    Must do this after configureUartTod or the configured
    delay will be overwritten.
    """
    # UBX-CFG-TP5
    # delay = 0 ns, RF group delay = 0 ns,
    # period = 1Hz, period (locked) = 10MHz,
    # pulseLen = 0, pulseLen (locked) = 50%,
    # active, lockGnssFreq, lockedOtherSet, isFreq,
    # alignToTow, polarity, gridUtcGnss = GPS, syncMode = 0
    cmd = array.array('B', [0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x01, 0x01,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                            0x00, 0x00, 0x80, 0x96, 0x98, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
                            0x00, 0x00, 0xEF, 0x00, 0x00, 0x00 ])

    delay += platform_delay()

    # Insert the antenna delay in little endian
    cmd[10] = delay & 0xff
    cmd[11] = (delay >> 8) & 0xff

    # Calculate and append the command checksum
    cmd += array.array('B', checksum(cmd))

    usb_dev._gps_set(cmd)


class _UbloxGNSS(GNSS):
    """
    ublox GNSS device support for the S9500 30XS platform.
    """
    MAX_RECORDS = 20

    def __init__(self):
        self.have_time = False
        self.gpstime = None
        self.have_position = False
        self.latitude = None
        self.longitude = None
        self.have_satellites = False
        self.GSV_decoded = 0
        self.satellites = []
        self.enabled = True
        self.in_holdover = False
        self.gnss_led = LedState.UNKNOWN
        self.sync_led = LedState.UNKNOWN
        self.antenna_status = None
        self.survey_status = None
        self.survey_observations = None
        self.survey_precision = None
        self.antenna_delay = 0

        syslog.openlog("vyatta-gnssd")

        # The GNSS might be stopped, ensure sure it is started
        # so that configuration will succeed.
        self.start()

    def start(self):
        """
        Start the ublox GNSS device.
        """
        def controlled_start(usb_dev):
            """
            Issue UBX-CFG-RST with resetMode = 0x9 (controlled start)
            """
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x09, 0x00, 0x17, 0x76])
            usb_write(usb_dev, cmd)

        def configure_gnss(usb_dev):
            """
            Apply the default configuration from UfiSpace's BSP
            """

            def disable_GSV(usb_dev):
                """
                Disable GSV NMEA sentences
                """
                cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                        0xf0, 0x03, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00])
                cmd += array.array('B', checksum(cmd))
                usb_dev._gps_set(cmd)

            def enable_GSV(usb_dev):
                """
                Enable GSV NMEA sentences
                """
                cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                        0xf0, 0x03, 0x00, 0x00, 0x00, 0x01,
                                        0x00, 0x01])
                cmd += array.array('B', checksum(cmd))
                usb_dev._gps_set(cmd)

            def enable_RMC(usb_dev):
                """
                Eisable RMC NMEA sentences
                """
                cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                        0xf0, 0x04, 0x01, 0x00, 0x00, 0x01,
                                        0x01, 0x01])
                cmd += array.array('B', checksum(cmd))
                usb_dev._gps_set(cmd)

            usb_dev.enable()
            disable_GSV(usb_dev)
            usb_dev.configureUartTod()
            enable_GSV(usb_dev)
            enable_RMC(usb_dev)

        def start_survey(usb_dev):
            """
            Start a survey with the UBX-CFG-TMODE2 command. The minimum
            duration for this survey is 5 minutes and the required
            accuracy for switching to fixed mode is 30 meters.
            """
            # UBX-CFG-TMODE2
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x3d, 0x1c, 0x00,
                                    0x01, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x2c, 0x01, 0x00, 0x00,  # 300s
                                    0x30, 0x75, 0x00, 0x00,  # 30m
                                    0x32, 0x0d])
            usb_dev._gps_set(cmd)

        usb_dev = GPSUSB()
        controlled_start(usb_dev)
        configure_gnss(usb_dev)
        ubx_cfg_tp5(usb_dev, self.antenna_delay)
        start_survey(usb_dev)

        self.enabled = True
        return True

    def stop(self):
        """
        Stop the ublox GNSS device.
        """
        def hardware_reset():
            """
            Issue UBX-CFG-RST with resetMode = 0x0 (hardware reset)
            """
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x0e, 0x64])
            usb_dev = GPSUSB()
            usb_write(usb_dev, cmd)

        def controlled_stop():
            """
            Issue UBX-CFG-RST with resetMode = 0x8 (controlled stop)
            """
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x08, 0x00, 0x16, 0x74])
            usb_dev = GPSUSB()
            usb_write(usb_dev, cmd)

        # The hardware takes approximately 2 seconds to
        # return from hardware reset.
        RESET_TIME = 2

        hardware_reset()

        # Wait for the hardware to come back. After it
        # returns, stop all the GNSS tasks.
        while True:
            try:
                time.sleep(RESET_TIME)
                controlled_stop()
                break
            except usb.USBError:
                pass
            except ValueError as e:
                pass

        self.enabled = False
        return True

    def set_antenna_delay(self, delay):
        """
        Set the antenna delay using the TimePulse2 command.
        Must do this after configure_gnss or the configured
        delay will be overwritten.
        """
        usb_dev = GPSUSB()
        ubx_cfg_tp5(usb_dev, delay)

        self.antenna_delay = delay
        return True

    def get_antenna_delay(self):
        return self.antenna_delay

    def get_status(self):
        """
        Get the status of ublox GNSS device.
        """
        is_shorted, is_open = gnss_antenna_state()
        antenna_status = gnss_antenna_status(is_shorted, is_open)

        tracking_status = 'unknown'
        (one_pps, ten_mhz) = gnss_dpll_state()
        if one_pps and ten_mhz:
            tracking_status = 'tracking'
        elif ten_mhz:
            tracking_status = 'acquiring'

        if (one_pps or ten_mhz) and antenna_status is 'open':
            antenna_status = 'OK'

        status = {'instance-number': self.get_instance(),
                  'antenna-status': antenna_status,
                  'antenna-delay': self.antenna_delay,
                  'enabled': self.enabled,
                  'system': 'gps-system',
                  'tracking-status': tracking_status}

        # If the GNSS is stopped, we won't be able to
        # fetch any additional information.
        if self.enabled:
            self.fetch_gnss_data()

            if self.have_time and self.gpstime:
                status['time'] = self.gpstime

            if self.have_position:
                if self.latitude:
                    status['latitude'] = str(self.latitude)
                if self.longitude:
                    status['longitude'] = str(self.longitude)

            if self.have_satellites:
                status['satellites-in-view'] = self.satellites

            if self.survey_status:
                status['survey-status'] = self.survey_status
                status['survey-observations'] = self.survey_observations
                if self.survey_precision:
                    status['survey-precision'] = self.survey_precision

        return status

    def decode_gsv(self, fields):
        """
        Decode a GSV NMEA sentence

        $GPGSV,3,1,12,01,22,253,23,07,04,268,19,08,80,281,12,10,51,072,*73
        $GPGSV,3,2,12,14,08,327,,16,16,176,12,21,48,259,23,22,14,203,20*75
        $GPGSV,3,3,12,23,23,046,,27,61,125,10,30,09,302,14,32,11,116,*74
        """
        if self.have_satellites:
            return

        num_sentences = int(fields[1])
        ith_sentence = int(fields[2])

        if ith_sentence == 1:
            self.GSV_decoded = 0
            self.satellites = []

        self.GSV_decoded = self.GSV_decoded + 1

        for index in range(4, len(fields), 4):
            satellite = {}
            satellite['instance-number'] = len(self.satellites)
            satellite['PRN'] = fields[index]

            elevation = fields[index + 1]
            if elevation != '':
                satellite['elevation'] = elevation
            azimuth = fields[index + 2]
            if azimuth != '':
                satellite['azimuth'] = azimuth
            snr = fields[index + 3]
            if snr != '':
                satellite['SNR'] = snr
            self.satellites.append(satellite)

        if num_sentences == self.GSV_decoded:
            self.have_satellites = True

    def decode_rmc(self, fields):
        """
        Decode a RMC NMEA sentence

        $GPRMC,144340.00,A,5127.30667,N,00058.67664,W,0.047,,020621,,,A*6D
        """
        if self.have_position:
            return

        # Is the position valid?
        if fields[3] == '' or fields[5] == '':
            self.have_position = True
            self.latitude = None
            self.longitude = None
            return

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
        if self.have_time:
            return

        # Is the time valid?
        if fields[1] == '' or fields[2] == '' or \
           fields[3] == '' or fields[4] == '':
            self.have_time = True
            self.gpstime = None
            return

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

    def fetch_svin(self, usb_dev):
        """
        Issue a UBX-CFG-SVIN to check the current status of survey-in.
        """
        def u4(bytes):
            """
            Convert a little endian Ublox U4 type to a native integer
            """
            if len(bytes) != 4:
                raise ValueError('Need exactly 4 bytes')
            return int(bytes[0]) + \
                int(bytes[1]) * 256 + \
                int(bytes[2]) * 65536 + \
                int(bytes[3]) * 16777216

        cmd = array.array('B', [0xb5, 0x62, 0x0d, 0x04, 0x00, 0x00,
                                0x11, 0x40])

        try:
            response = usb_dev._gps_get(cmd)

            variance = u4(response[22:26])
            observations = u4(response[26:30])
            valid = int(response[30])
            active = int(response[31])
        except usb.USBError:
            valid = 0
            active = 0

        if active:
            self.survey_status = 'sampling'
        elif valid:
            self.survey_status = 'fixed-position'
        else:
            self.survey_status = None

        if active or valid:
            self.survey_observations = observations
        else:
            self.survey_observations = 0

        if self.survey_observations == 0:
            self.survey_precision = None
        else:
            self.survey_precision = math.sqrt(variance)

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
        self.GSV_decoded = 0

        records_read = 0

        read = True
        while read:
            try:
                resp = endpoint.read(POLL_TIMEOUT)
                try:
                    self.decode_sentence(resp.tobytes().decode('utf-8'))
                except UnicodeDecodeError:
                    continue

                if self.have_time and \
                   self.have_position and \
                   self.have_satellites:
                    read = False

                records_read += 1
                if records_read > self.MAX_RECORDS:
                    read = False
            except usb.USBError:
                # Continuous read until timeout
                pass

        self.fetch_svin(usb_dev)

    def update_hardware_status(self):
        """
        Update the GPS and SYNC status LEDs.
        """
        def update_led(led, state):
            """
            Set the given LED to the specific state
            """
            state_table = {
                LedState.OFF:
                    (Led.STATUS_OFF,
                     Led.COLOR_GREEN,
                     Led.BLINK_STATUS_SOLID),
                LedState.GREEN:
                    (Led.STATUS_ON,
                     Led.COLOR_GREEN,
                     Led.BLINK_STATUS_SOLID),
                LedState.BLINKING_GREEN:
                    (Led.STATUS_ON,
                     Led.COLOR_GREEN,
                     Led.BLINK_STATUS_BLINKING),
                LedState.YELLOW:
                    (Led.STATUS_ON,
                     Led.COLOR_YELLOW,
                     Led.BLINK_STATUS_SOLID),
                LedState.BLINKING_YELLOW:
                    (Led.STATUS_ON,
                     Led.COLOR_YELLOW,
                     Led.BLINK_STATUS_BLINKING),
            }

            on_off, color, blink = state_table[state]

            cpld_util = CPLDUtility()
            cpld_util.set_led_control(led, on_off, color, blink)

        is_shorted, is_open = gnss_antenna_state()

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

        if not self.enabled:
            state = LedState.OFF
        elif is_open == 0 and is_shorted == 0:
            state = LedState.BLINKING_YELLOW
        elif is_open == 0 and is_shorted == 1:
            (one_pps, ten_mhz) = gnss_dpll_state()

            if one_pps and ten_mhz:
                state = LedState.GREEN
            elif ten_mhz:
                state = LedState.BLINKING_GREEN
            else:
                state = LedState.YELLOW
        elif is_open == 1 and is_shorted == 1:
            (one_pps, ten_mhz) = gnss_dpll_state()

            if one_pps and ten_mhz:
                state = LedState.GREEN
            elif ten_mhz:
                state = LedState.BLINKING_GREEN   # frequency input
            else:
                state = LedState.OFF
        else:
            state = LedState.OFF

        if self.gnss_led != state:
            update_led(Led.GPS, state)
            self.gnss_led = state

        # This platform has a stratum 3e clock. It is precise
        # enough to maintain end-to-end PTP timing for at least
        # 2 hours.
        IN_SPEC_HOLDOVER_TIME = 7200    # seconds

        timing_state = dpll_get_timing_state()
        if timing_state == TimingState.PHASE_LOCKED:
            state = LedState.GREEN
            self.in_holdover = False
        elif timing_state == TimingState.FREQUENCY_LOCKED:
            state = LedState.BLINKING_GREEN
            self.in_holdover = False
        elif timing_state == TimingState.HOLDOVER:
            # If we have been in holdover too long, then we
            # should blink to indicate that we have exceeded
            # the holdover specification.
            if not self.in_holdover:
                self.in_holdover = True
                self.holdover_start = time.time()

            holdover_time = time.time() - self.holdover_start
            if holdover_time > IN_SPEC_HOLDOVER_TIME:
                state = LedState.BLINKING_YELLOW
            else:
                state = LedState.YELLOW
        else:
            state = LedState.OFF
            self.in_holdover = False

        if self.sync_led != state:
            update_led(Led.SYNC, state)
            self.sync_led = state

        # Log antenna status update if necessary
        antenna_status = gnss_antenna_status(is_shorted, is_open)
        if antenna_status is 'open':
            (one_pps, ten_mhz) = gnss_dpll_state()
            if (one_pps or ten_mhz):
                antenna_status = 'OK'

        if antenna_status != self.antenna_status:
            instance_number = self.get_instance()
            syslog.syslog(syslog.LOG_WARNING,
                          f'GNSS {instance_number}: antenna status has changed to {antenna_status}')
            client = vci.Client()
            notification = {
                'antenna-status': antenna_status,
                'instance-number': instance_number
            }
            client.emit('vyatta-service-gnss-v1',
                        'antenna-status-update', notification)
            self.antenna_status = antenna_status


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
