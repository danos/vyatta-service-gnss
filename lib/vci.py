# Copyright (c) 2021, Ciena Corporation.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
A simple emulation of vci for unit testing
"""

import time


class Exception(Exception):
    def __init__(self):
        pass

class Client:
    def __init__(self):
        raise Exception()
