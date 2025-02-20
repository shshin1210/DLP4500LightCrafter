import pycrafter4500

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import time
import usb.util

import numpy as np

# power on projector
pycrafter4500.power_up()

# Start pattern projection
pycrafter4500.pattern_sequence_final(
    input_mode='pattern',
    input_source='flash',
    num_pats=8,
    trigger_type='pattern_trigger_mode1',
    exposure_period=100000,
    frame_period=100000,
    bit_depth=[1,1,1,1,1,1,1,1],
    led_color=0b111,
    trig_type=0
)

pycrafter4500.power_down()