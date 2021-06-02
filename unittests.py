#!/bin/python3

# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only


"""
Unit testing for vyatta-gnssd
"""

import subprocess
import time
import unittest


class TestGNSS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        subprocess.Popen(['sbin/vyatta-gnssd'], env={'PYTHONPATH': './lib'})
        time.sleep(1)

    def test_get_state(self):
        output = subprocess.check_output(
            ['bin/vyatta-gnss-util', '--get-state'])
        self.assertNotEqual(output, '')

    def test_do_stop(self):
        subprocess.check_output(['bin/vyatta-gnss-util', '--stop-gnss'])
        self.assertTrue(True, 'subprocess will raise an exception on failure')

    def test_do_start(self):
        subprocess.check_output(['bin/vyatta-gnss-util', '--start-gnss'])
        self.assertTrue(True, 'subprocess will raise an exception on failure')

    @classmethod
    def tearDownClass(cls):
        subprocess.check_output(['bin/vyatta-gnss-util', '--shutdown'])


class TestOpmode(unittest.TestCase):

    def test_do_show(self):
        output = subprocess.check_output(
            ['bin/vyatta-gnss-op', '--action=show'], env={'PYTHONPATH': './lib'})
        self.assertNotEqual(output, '')


if __name__ == '__main__':
    unittest.main()
