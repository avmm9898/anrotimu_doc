#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' a test module '

from hipnuc_module import *
import time


if __name__ == '__main__':

    #my_IMU 變數名稱
    my_IMU = hipnuc_module('./config.json')
    
    while True:
        try:
            data = my_IMU.get_module_data()
        except:
            my_IMU.close()
            break
            
        #id, timestamp, acc, gyr, quat, id, linacc, float_eul, int_eul
        print(data)
        #print(data['int_eul'])