#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from hipnuc_module import *
import time

if __name__ == '__main__':

    # m_IMU_serial= class of hipnuc_module 
    m_IMU_serial = hipnuc_module('./config.json')

    while True:
        try:
            data = m_IMU_serial.get_module_data(1)
        except:
            print("Serial closed.")
            m_IMU_serial.close()
            break

        try:
            #this prints all : print(data)#

            #id, timestamp, acc, gyr, quat, mag, euler#
            #print(data)
            pass
        except:
            print("Print error.")
            m_IMU_serial.close()
            break
