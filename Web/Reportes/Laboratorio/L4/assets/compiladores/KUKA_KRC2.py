# -*- coding: UTF-8 -*-
# Copyright 2015-2025 - RoboDK Inc. - https://robodk.com/
#
# This is a compiled post processor. Please contact us at info@robodk.com if you need access to the source code of this post processor.
#
# This file loads the compiled version of the RoboDK post processor for:
#   KUKA KRC2 robot controllers
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
exec("from v" + version_str + ".KUKA_KRC2 import RobotPost as BasePost")

class RobotPost(BasePost):

    # ------------------------ Customize your RoboDK post processor using the following variables ------------------------

    # Maximum number of lines per program. It will then generate multiple "pages (files)" if the program is too long
    # This is the default value. You can change this value in RoboDK:
    # Tools-Options-Program-Limit the maximum number of lines per program
    MAX_LINES_X_PROG = 2500

    # Set to True to include subprograms in the main program. Otherwise, programs can be generated separately.
    INCLUDE_SUB_PROGRAMS = False    # Set to True to include subprograms in the same file

    # Generate subprograms as new files instead of adding them in the same file (INCLUDE_SUB_PROGRAMS must be set to True)
    SUB_PROGRAMS_AS_FILES = True

    # Set KRC Version (2 or 4)
    # Version 2 for KRC2 controllers or Version 4 for KRC4
    # KRC_VERSION = 4  # for KRC4 controllers
    KRC_VERSION = 2    # for KRC2 controllers

    # Display messages on the teach pendant. This feature is not supported by all controllers and must be disabled for some controllers.
    # SKIP_MESSAGE_POPUPS = True
    SKIP_MESSAGE_POPUPS = False

    # Use the Frame index instead of the pose when the index is provided
    # Example: Set to True and rename your reference to "Frame 2" to use BASE_DATA[2])
    BASE_INDEX = False

    # Use the Tool index instead of the pose when the index is provided
    # Example: Set to True and rename your tool to "Tool 3" to use TOOL_DATA[3])
    TOOL_INDEX = False

    # Default BASE ID to use when the base index is not specified in the reference name and BASE_INDEX is set to True
    DEFAULT_BASE_ID = 1

    # Default TOOL ID to use when the tool index is not specified in the reference name and TOOL_INDEX is set to True
    DEFAULT_TOOL_ID = 1

    # Define the move linear keyword (usually LIN)
    # Other examples: SLIN, SPL (use a Spline Block, ...
    # Tip: Add a comment such as start_spl, start_lin or start_slin to change the move command
    LIN_KEYWORD = "LIN"

    # Use SPLINE block (SPL movements). You can change this flag or change the LIN KEYWORD to SPL.
    SPLINE_BLOCK = False

    # Set if we want to have cascaded program calls when program splitting takes place
    # Usually, cascaded is better for older controllers
    # CASCADED_CALLS = True
    # CASCADED_CALLS = False
    CASCADED_CALLS = True

    # Enter program names to exclude from EXT definition in each program header
    # PROG_EXT_EXCLUDE = ["tool_change","spindle_start","spindle_stop"]
    # PROG_EXT_EXCLUDE = True # Never add EXT programs in the header
    # PROG_EXT_EXCLUDE = False # Always add EXT programs
    PROG_EXT_EXCLUDE = False

    # Optionally output configuration and turn flags S and T
    # If we set this to true, the output targets will have the configuration and turn bits added in the movement
    # (for example: S B'010', T B'000001')
    ADD_CONFIG_TURN = False

    # Add a keyword to split subprograms
    # SKIP_PROG_KEYWORD = 'spindle'
    SKIP_PROG_KEYWORD = None

    # Set the name/order of default axes and external axes
    AXES_DATA = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'E1', 'E2', 'E3', 'E4', 'E5', 'E6']

    # Program name max length (KRC4 is 24 characters)
    PROG_NAME_LEN = 16

    # Parameters for 3D printing
    # Use a synchronized external axis as an extruder (E1, E2, ... or the last available axis)
    EXTAXIS_EXTRUDER = False

    # Use a weld gun as an extruder
    # WELD_EXTRUDER = True
    WELD_EXTRUDER = False

    # Parameters for 3D printing using a custom extruder
    # 3D Printing Extruder Setup Parameters:
    PRINT_E_ANOUT = 5    # Analog Output ID to command the extruder flow

    PRINT_SPEED_2_SIGNAL = 0.1    # Ratio to convert the speed/flow to an analog output signal

    PRINT_FLOW_MAX_SIGNAL = 24    # Maximum signal to provide to the Extruder

    PRINT_ACCEL_MMSS = -1    # Acceleration, -1 assumes constant speed if we use rounding/blending

    # Set or change the main header here
    # Example: HEADER = DEFAULT_HEADER + "CallProgram();\n"
    HEADER = """

; GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
; INTERRUPT ON 3

;FOLD Initialise and set default speed
BAS (#INITMOV,0)
BAS (#VEL_PTP,100)
BAS (#ACC_PTP,20)
$VEL.CP=0.2
BAS (#TOOL,0)
BAS (#BASE,0)
;ENDFOLD

;;FOLD STARTPOS
;$BWDSTART = FALSE
;PDAT_ACT = PDEFAULT
;BAS(#PTP_DAT)
;FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
;BAS(#FRAMES)
;;ENDFOLD

$ADVANCE = 5

;FOLD ---- Quickly skip BCO ----
; PTP $AXIS_ACT
;ENDFOLD

;FOLD ---- GO HOME ----
; PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000, E1 0, E2 0, E3 0, E4 0, E5 0, E6 0}
;ENDFOLD

"""

    # --------------------------------------------------------------------------------------------------------------------

    pass

if __name__== "__main__":
    exec("from v" + version_str + ".KUKA_KRC2 import test_post")
    test_post()

