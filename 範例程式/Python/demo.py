#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' a test module '

from hipnuc_module import *
import time


if __name__ == '__main__':

    HI221GW = hipnuc_module('./config.json')

    while True:
        data = HI221GW.get_module_data()

        # print(data)

        time.sleep(0.010)