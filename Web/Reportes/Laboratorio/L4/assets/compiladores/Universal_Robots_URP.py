# -*- coding: UTF-8 -*-
# Copyright 2015-2025 - RoboDK Inc. - https://robodk.com/
#
# This is a compiled post processor. Please contact us at info@robodk.com if you need access to the source code of this post processor.
#
# This file loads the compiled version of the RoboDK post processor for:
#   Universal Robots URP robot controllers
#
# More information about RoboDK Post Processors and Offline Programming:
#     https://robodk.com/help#PostProcessor
#     https://robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------

import sys
import os

import math
from robodk import *

# Detect Python version and post processor
print("Using Python version: " + str(sys.version_info))
path_file = os.path.dirname(__file__).replace(os.sep, "/")
print("RoboDK Post Processor: " + path_file)

# Check if the post is compatible with the Python version
version_str = str(sys.version_info[0]) + str(sys.version_info[1])
path_library = path_file + '/v' + version_str
if not os.path.isdir(path_library):
    msg = "Invalid Python version or post processor not found. Make sure you are using a supported Python version: " + path_library
    msg += "\nSelect Tools-Options-Python and select a supported Python version"
    print(msg)
    raise Exception(msg)

# Load the post processor
exec("from v" + version_str + ".Universal_Robots_URP import RobotPost as BasePost")

class RobotPost(BasePost):

    # ------------------------ Customize your RoboDK post processor using the following variables ------------------------

    # Set to True to use MoveP, set to False to use MoveL
    USE_MOVEP = False

    # If True, it will attempt to upload using SFTP. It requires PYSFTP (pip install pysftp. Important: It requires Visual Studio Community C++ 10.0)
    UPLOAD_SFTP = False

    # default speed for linear moves in m/s
    SPEED_MS = 0.25

    # default speed for joint moves in rad/s
    SPEED_RADS = 0.75

    # default acceleration for lineaer moves in m/ss
    ACCEL_MSS = 1.2

    # default acceleration for joint moves in rad/ss
    ACCEL_RADSS = 1.2

    # default blend radius in meters (corners smoothing)
    # BLEND_RADIUS_M = 0.001
    BLEND_RADIUS_M = 0.0

    # 5000    # Maximum number of lines per program. If the number of lines is exceeded, the program will be executed step by step by RoboDK
    MAX_LINES_X_PROG = 1000000000.0

    # minimum circle radius to output (in mm). It does not take into account the Blend radius
    MOVEC_MIN_RADIUS = 1

    # maximum circle radius to output (in mm). It does not take into account the Blend radius
    MOVEC_MAX_RADIUS = 10000

    # Maximum speed allowed by the controller (otherwise it throws a speed error)# Maximum speeds and accelerations allowed by the controller (otherwise it throws a speed error)
    MAX_SPEED_MS = 3.0

    MAX_SPEED_DEGS = 180

    MAX_ACCEL_MSS = 15.0

    MAX_ACCEL_DEGSS = 2291.8

    # Prevent generating script code for joint moves (may create problems related to configuration)
    SKIP_MOVEJ = False

    # --------------------------------------------------------------------------------------------------------------------

    pass

if __name__== "__main__":
    exec("from v" + version_str + ".Universal_Robots_URP import test_post")
    test_post()

