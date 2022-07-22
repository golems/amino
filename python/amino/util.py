#  Copyright (c) 2019, Colorado School of Mines
#  All rights reserved.
#
#  Author(s): Neil T. Dantam <ndantam@mines.edu>
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
"""Utilities"""

import sys

if sys.version_info[0] < 3:
    raise Exception("Python 3 or a more recent version is required.")

def is_int(thing):
    """Returns True when thing is an integer type."""
    return isinstance(thing, int)

def is_string(thing):
    """Returns True when thing is a string type."""
    return isinstance(thing, str)

def ensure_cstring(pystring):
    """Return a string we can pass to C"""
    if isinstance(pystring, bytes):
        return pystring
    return pystring.encode('utf-8')

def ensure(thing, desired_type):
    """If thing is not of desired_type, construct a new desired_type from thing"""
    return thing if isinstance(thing, desired_type) else desired_type(thing)

def is_scalar(thing):
    """Returns True when thing is an integer or float type."""
    return is_int(thing) or isinstance(thing, float)
