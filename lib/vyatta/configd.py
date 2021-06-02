# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
A simple emulation for unit testing
"""

import time


class Exception(Exception):
    def __init__(self):
        pass


class Client:
    def __init__(self):
        pass

    def tree_get_full_dict(self, path):
        if path != 'service gnss state':
            return None

        return {'state': {
            'instance-list': [
                {'instance-number': 0,
                 'antenna-status': 'OK',
                 'time': time.time(),
                 'enabled': True,
                 'latitude': 0.0,
                 'longitude': 0.0,
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
                 'tracking-status': 'tracking'}
            ]
        }
        }
