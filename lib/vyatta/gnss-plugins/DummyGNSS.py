#!/usr/bin/python3

# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
A simple GNSS device used for unit testing
"""

import time
import random

from vyatta.gnss import GNSS


class DummyGNSS(object):
    def __init__(self):
        pass

    def probe(self, register):
        instance = _DummyGNSS()
        register(instance)
        print('Adding DummyGNSS instance', instance.get_instance())


class _DummyGNSS(GNSS):
    def __init__(self):
        self.enabled = True

    def start(self):
        self.enabled = True
        return True

    def stop(self):
        self.enabled = False
        return True

    def get_status(self):
        lat = random.randrange(-179, 179) + random.random()
        long = random.randrange(-179, 179) + random.random()

        track_status = random.choice(['acquiring', 'tracking', 'unknown'])
        ant_status = random.choice(['OK', 'short', 'open', 'unknown'])

        return {'instance-number': self.get_instance(),
                'antenna-status': ant_status,
                'time': time.time(),
                'enabled': self.enabled,
                'latitude': str(lat),
                'longitude': str(long),
                'satellites-in-view': [
                    {'instance-number': 1, 'PRN':  '1',
                        'elevation': 59, 'azimuth': 281, 'SNR': 26},
                    {'instance-number': 2, 'PRN':  '3',
                        'elevation': 25, 'azimuth': 215, 'SNR': 28},
                    {'instance-number': 3, 'PRN':  '8',
                        'elevation': 54, 'azimuth': 159, 'SNR': 10},
                    {'instance-number': 4, 'PRN': '10',
                        'elevation': 20, 'azimuth': 50},
        ],
            'system': 'gps-system',
            'tracking-status': track_status}

    def update_hardware_status(self):
        print('Time to update the LEDs!')
