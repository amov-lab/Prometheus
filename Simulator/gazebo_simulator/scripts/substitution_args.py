#!/usr/bin/env python3
"""
Library for processing XML substitution args.
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
# Revision $Id: substitution_args.py 15178 2011-10-10 21:22:53Z kwc $

from io import StringIO
import os


class SubstitutionException(Exception):
    """
    Base class for exceptions in substitution_args routines
    """
    pass


class ArgException(SubstitutionException):
    """
    Exception for missing $(arg) values
    """
    pass


def _split_command(resolved, command_with_args):
    cmd = '$(%s)' % command_with_args
    idx1 = resolved.find(cmd)
    idx2 = idx1 + len(cmd)
    return resolved[0:idx1], resolved[idx2:]


def _separate_first_path(value):
    idx = value.find(' ')
    if idx < 0:
        path, rest = value, ''
    else:
        path, rest = value[0:idx], value[idx:]
    return path, rest


def _sanitize_path(path):
    path = path.replace('/', os.sep)
    path = path.replace('\\', os.sep)
    return path


def _arg(resolved, a, args, context):
    """
    process $(arg) arg

    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException(
            "$(arg var) must specify an environment variable [%s]" %
            (a))
    elif len(args) > 1:
        raise SubstitutionException(
            "$(arg var) may only specify one arg [%s]" %
            (a))

    if 'arg' not in context:
        context['arg'] = {}
    arg_context = context['arg']

    arg_name = args[0]
    if arg_name in arg_context:
        arg_value = arg_context[arg_name]
        return resolved.replace("$(%s)" % a, arg_value)
    else:
        raise ArgException(arg_name)


def resolve_args(arg_str, context=None, resolve_anon=True):
    """
    Resolves substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args
        in. arg_str may be None, in which case resolve_args will
        return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of
        the 'anon' and 'arg' substitution args. multiple calls to
        resolve_args should use the same context so that 'anon'
        substitions resolve consistently. If no context is provided, a
        new one will be created for each call. Values for the 'arg'
        context should be stored as a dictionary in the 'arg' key.
    @type  context: dict
    @param resolve_anon bool: If True (default), will resolve $(anon
        foo). If false, will leave these args as-is.
    @type  resolve_anon: bool

    @return str: arg_str with substitution args resolved
    @rtype:  str
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    # parse found substitution args
    if not arg_str:
        return arg_str
    # first resolve variables like 'env' and 'arg'
    commands = {
        'arg': _arg,
    }
    resolved = _resolve_args(arg_str, context, resolve_anon, commands)
    # than resolve 'find' as it requires the subsequent path to be expanded
    # already

    resolved = _resolve_args(resolved, context, resolve_anon, commands)
    return resolved


def _resolve_args(arg_str, context, resolve_anon, commands):
    valid = ['arg']
    resolved = arg_str
    for a in _collect_args(arg_str):
        splits = [s for s in a.split(' ') if s]
        if not splits[0] in valid:
            raise SubstitutionException(
                "Unknown substitution command [%s]. Valid commands are %s" %
                (a, valid))
        command = splits[0]
        args = splits[1:]
        if command in commands:
            resolved = commands[command](resolved, a, args, context)
    return resolved


_OUT = 0
_DOLLAR = 1
_LP = 2
_IN = 3


def _collect_args(arg_str):
    """
    State-machine parser for resolve_args. Substitution args are of the form:
    $(find package_name)/scripts/foo.py $(export some/attribute blar) non-relevant stuff

    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: list of arguments
    @rtype: [str]
    """
    buff = StringIO()
    args = []
    state = _OUT
    for c in arg_str:
        # No escapes supported
        if c == '$':
            if state == _OUT:
                state = _DOLLAR
            elif state == _DOLLAR:
                pass
            else:
                raise SubstitutionException(
                    "Dollar signs '$' cannot be inside of substitution args [%s]" %
                    arg_str)
        elif c == '(':
            if state == _DOLLAR:
                state = _LP
            elif state != _OUT:
                raise SubstitutionException(
                    "Invalid left parenthesis '(' in substitution args [%s]" % arg_str)
        elif c == ')':
            if state == _IN:
                # save contents of collected buffer
                args.append(buff.getvalue())
                buff.truncate(0)
                buff.seek(0)
                state = _OUT
            else:
                state = _OUT
        elif state == _DOLLAR:
            # left paren must immediately follow dollar sign to enter _IN state
            state = _OUT
        elif state == _LP:
            state = _IN

        if state == _IN:
            buff.write(c)
    return args
