#!/usr/bin/env python3
"""
Library for manipulating ROS Names. See U{http://ros.org/wiki/Names}.
"""

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: names.py 14589 2011-08-07 18:30:21Z kwc $


import os
import sys

SEP = '/'
GLOBALNS = '/'
PRIV_NAME = '~'
REMAP = ":="
ANYTYPE = '*'

def load_mappings(argv):
    """
    Load name mappings encoded in command-line arguments. This will filter
    out any parameter assignment mappings.

    @param argv: command-line arguments
    @type  argv: [str]
    @return: name->name remappings.
    @rtype: dict {str: str}
    """
    mappings = {}
    for arg in argv:
        if REMAP in arg:
            try:
                src, dst = [x.strip() for x in arg.split(REMAP)]
                if src and dst:

                    if len(src) > 1 and src[0] == '_' and src[1] != '_':
                        # ignore parameter assignment mappings
                        pass
                    else:
                        mappings[src] = dst
            except BaseException:
                #TODO: remove
                sys.stderr.write(
                    "ERROR: Invalid remapping argument '%s'\n" %
                    arg)
    return mappings
