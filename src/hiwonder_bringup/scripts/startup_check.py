#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/04
# @author:aiden
# 自检程序(self-test program)
import os
import re
import time
import smbus
from hiwonder_sdk import buzzer, button

def check_mic():
    data = os.popen('lsusb').read()
    res = re.findall(r'10d6:b003', data)
    if res != []:
        os.system("roslaunch xf_mic_asr_offline startup_test.launch")

def check_sensor():
    bus = smbus.SMBus(1)
    count_imu = 0
    while True:
        count_imu += 1
        try:
            bus.write_byte_data(0x68, 0x6b, 0)
            if button.get_button_status('key1') == 1 and button.get_button_status('key2') == 1:
                time.sleep(12)
                buzzer.on()
                time.sleep(0.1)
                buzzer.off()
                print('everything is ok!')
            break
        except Exception as e:
            print(str(e))
        if count_imu > 100:
            count_imu = 0 
            print('imu init timeout')
            break
        time.sleep(0.01)

if __name__ == '__main__':
    check_sensor()
    check_mic()
