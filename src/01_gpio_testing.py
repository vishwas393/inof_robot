#!/usr/bin/env python

import RPi.GPIO as gpio
import time

int LED = 26;

def setup_led():
    gpio.setmode(GPIO.BCM)
    gpio.setup(LED, gpio.OUT);
    print "LED SET! \n"


def blink_led():
    while(1):
        gpio.output(LED, gpio.HIGH)
        time.sleep(1)
        gpio.output(LED, gpio.LOW)
        time.sleep(1);
    gpio.cleanup()

if __name__ == '__main__':
    setup_led()
    blink_led()
