#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Basic Read Sensor Data example

This example shows how to retrieve data from a sensor channel

Requires at least one Fluigent sensor (Flow Unit or IPS)

Copyright (c) Fluigent 2020.  All Rights Reserved.
"""

import time

from Fluigent.SDK import fgt_init, fgt_close
from Fluigent.SDK import fgt_get_sensorChannelsInfo
from Fluigent.SDK import fgt_get_sensorUnit, fgt_get_sensorRange, fgt_get_sensorValue

## Initialize the session
# This step is optional, if not called session will be automatically created
fgt_init()

## Get information about sensors and read them

sensorInfoArray, sensorTypeArray = fgt_get_sensorChannelsInfo()


for i, sensorInfo in enumerate(sensorInfoArray):
    print('Sensor channel info at index: {}'.format(i))
    print(sensorInfo)
    print("Sensor type: {}".format(sensorTypeArray[i]))
    # Get measurement unit
    unit = fgt_get_sensorUnit(i)
    # Get sensor range
    minSensor, maxSensor = fgt_get_sensorRange(i)
    print("Range {:0.2f} to {:0.2f} {}".format(minSensor, maxSensor, unit))


for i, sensorInfo in enumerate(sensorInfoArray):
    unit = fgt_get_sensorUnit(i)
    measurement = fgt_get_sensorValue(i)
    print('Sensor channel index {} Measured {:0.2f} {}'.format(i, measurement, unit))
    time.sleep(0.2)


## Close the session
fgt_close()
