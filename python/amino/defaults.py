#  Copyright (c) 2020, Colorado School of Mines
#  All rights reserved.
#
#  Author(s): Matthew A. Schack <mschack@mines.edu>
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
#   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
#   TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
#   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
#   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
#   SUCH DAMAGE.

##
## @file defaults.py defaults
##
"""Default return values"""
import ctypes
from amino.lib import libamino

## Status code for OK
AA_RX_OK = ctypes.c_int.in_dll(libamino, "aa_rx_ok").value
## Status code for No Solution
AA_RX_NO_SOLUTION = ctypes.c_int.in_dll(libamino, "aa_rx_no_solution").value
## Status code for No Inverse Kinematics Solution
AA_RX_NO_IK = ctypes.c_int.in_dll(libamino, "aa_rx_no_ik").value
## Status code for No Motion Plan
AA_RX_NO_MP = ctypes.c_int.in_dll(libamino, "aa_rx_no_mp").value
## Status code for Invalid frame
AA_RX_INVALID_FRAME = ctypes.c_int.in_dll(libamino, "aa_rx_invalid_frame").value
## Status code for Invalid Parameter
AA_RX_INVALID_PARAMETER = ctypes.c_int.in_dll(libamino, "aa_rx_invalid_parameter").value
## Status code for Invalid State
AA_RX_INVALID_STATE = ctypes.c_int.in_dll(libamino, "aa_rx_invalid_state").value
