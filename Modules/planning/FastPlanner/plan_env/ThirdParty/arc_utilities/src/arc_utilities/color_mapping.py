#!/usr/bin/env python

import math
import numpy


def convert_rgb_to_xyz(r, g, b):
    rgb_np = numpy.array([r, g, b])
    rgb_xyz_cvt_np = numpy.array([[0.4124, 0.2126, 0.0193], [0.3576, 0.7152, 0.1192], [0.1805, 0.0722, 0.9505]])
    xyz_np = numpy.dot(rgb_np, rgb_xyz_cvt_np)
    return [xyz_np[0], xyz_np[1], xyz_np[2]]


def convert_xyz_to_rgb(x, y, z):
    xyz_np = numpy.array([x, y, z])
    rgb_xyz_cvt_np = numpy.array([[0.4124, 0.2126, 0.0193], [0.3576, 0.7152, 0.1192], [0.1805, 0.0722, 0.9505]])
    xyz_rgb_cvt_np = numpy.linalg.inv(rgb_xyz_cvt_np)
    rgb_np = numpy.dot(xyz_np, xyz_rgb_cvt_np)
    return [rgb_np[0], rgb_np[1], rgb_np[2]]


def cielab_f(val):
    if val > (216.0 / 24389.0):
        return math.pow(val, (1.0 / 3.0))
    else:
        return (841.0 / 108.0) * val + (16.0 / 116.0)


def cielab_finv(val):
    if val > (6.0 / 29.0):
        return val ** 3
    else:
        return (108.0 / 841.0) * (val - (16.0 / 116.0))


def get_ref_xyz():
    xref = 95.047
    yref = 100.0
    zref = 108.883
    return [xref, yref, zref]


def convert_xyz_to_lab(x, y, z):
    [xref, yref, zref] = get_ref_xyz()
    l = 116.0 * (cielab_f(y / yref) - (16.0 / 116.0))
    a = 500.0 * (cielab_f(x / xref) - cielab_f(y / yref))
    b = 200.0 * (cielab_f(y / yref) - cielab_f(z / zref))
    return [l, a, b]


def convert_lab_to_xyz(l, a, b):
    intermediate = (l + 16.0) * (1.0 / 116.0)
    x = cielab_finv(intermediate + a * (1.0 / 500.0))
    y = cielab_finv(intermediate)
    z = cielab_finv(intermediate - b * (1.0 / 200.0))
    return [x, y, z]


def convert_lab_to_msh(l, a, b):
    m = math.sqrt(l ** 2 + a ** 2 + b ** 2)
    s = math.acos(l / m)
    #h = math.atan2(b, a)
    h = math.atan(b / a)
    return [m, s, h]


def convert_msh_to_lab(m, s, h):
    l = m * math.cos(s)
    a = m * math.sin(s) * math.cos(h)
    b = m * math.sin(s) * math.sin(h)
    return [l, a, b]


def rgb_to_msh(r, g, b):
    [x, y, z] = convert_rgb_to_xyz(r, g, b)
    [l, a, b] = convert_xyz_to_lab(x, y, z)
    [m, s, h] = convert_lab_to_msh(l, a, b)
    return [m, s, h]


def msh_to_rgb(m, s, h):
    [l, a, b] = convert_msh_to_lab(m, s, h)
    [x, y, z] = convert_lab_to_xyz(l, a, b)
    [r, g, b] = convert_xyz_to_rgb(x, y, z)
    return [r, g, b]


def rad_diff(x, y):
    diff = abs(x - y)
    if diff > math.pi:
        return (math.pi * 2.0) - diff
    else:
        return diff


def adjust_hue((m_saturated, s_saturated, h_saturated), m_unsaturated):
    if m_saturated >= m_unsaturated:
        return h_saturated
    else:
        h_spin = (s_saturated * math.sqrt(m_unsaturated ** 2 - m_saturated ** 2)) / (m_saturated * math.sin(s_saturated))
        if h_saturated > -(math.pi / 3.0):
            return h_saturated + h_spin
        else:
            return h_saturated - h_spin


def interpolate_coolwarm((r1, g1, b1), (r2, g2, b2), interpolation):
    assert(0.0 <= interpolation <= 1.0)
    [m1, s1, h1] = rgb_to_msh(r1, g1, b1)
    [m2, s2, h2] = rgb_to_msh(r2, g2, b2)
    # If points saturated and distinct, place white in middle
    if (s1 > 0.05) and (s2 > 0.05) and (rad_diff(h1, h2) > (math.pi / 3.0)):
        m_mid = max((m1, m2, 88.0))
        if interpolation < 0.5:
            m2 = m_mid
            s2 = 0.0
            h2 = 0.0
            interpolation *= 2.0
        else:
            m1 = m_mid
            s1 = 0.0
            h1 = 0.0
            interpolation = 2.0 * interpolation - 1.0
    # Adjust hue of unsaturated colors
    if (s1 < 0.05) and (s2 > 0.05):
        h1 = adjust_hue((m2, s2, h2), m1)
    elif (s2 < 0.05) and (s1 > 0.05):
        h2 = adjust_hue((m1, s1, h1), m2)
    # Linear interpolation on adjusted control points
    m_mid = ((1.0 - interpolation) * m1) + (interpolation * m2)
    s_mid = ((1.0 - interpolation) * s1) + (interpolation * s2)
    h_mid = ((1.0 - interpolation) * h1) + (interpolation * h2)
    return msh_to_rgb(m_mid, s_mid, h_mid)


def jet_interpolate(value, y0, x0, y1, x1):
    return (value - x0) * (y1 - y0) / (x1 - x0) + y0


def jet_base(value):
    if value <= -0.75:
        return 0.0
    elif value <= -0.25:
        return jet_interpolate(value, 0.0, -0.75, 1.0, -0.25)
    elif value <= 0.25:
        return 1.0
    elif value <= 0.75:
        return jet_interpolate(value, 1.0, 0.25, 0.0, 0.75)
    else:
        return 0.0


def interpolate_jet(value, use_negative_range=False):
    if use_negative_range:
        assert(-1.0 <= value <= 1.0)
        r = jet_base(value - 0.5)
        g = jet_base(value)
        b = jet_base(value + 0.5)
        return [r, g, b]
    else:
        assert(0.0 <= value <= 1.0)
        if value > 0.5:
            value = (value - 0.5) * 2.0
        elif value < 0.5:
            value = -(0.5 - value) * 2.0
        else:
            value = 0.0
        r = jet_base(value - 0.5)
        g = jet_base(value)
        b = jet_base(value + 0.5)
        return [r, g, b]


def interpolate_hot_to_cold(value, min_value=0.0, max_value=1.0):
    # Safety checks
    assert(min_value < max_value)
    if value < min_value:
        value = min_value
    elif value > max_value:
        value = max_value
    val_range = max_value - min_value
    # Start with white
    r = 1.0
    g = 1.0
    b = 1.0
    # Interpolate
    if value < (min_value + 0.25 * val_range):
        r = 0.0
        g = 4.0 * (value - min_value) / val_range
    elif value < (min_value + 0.5 * val_range):
        r = 0.0
        b = 1.0 + 4.0 * (min_value + 0.25 * val_range - value) / val_range
    elif value < (min_value + 0.75 * val_range):
        r = 4 * (value - min_value - 0.5 * val_range) / val_range
        b = 0.0
    else:
        g = 1.0 + 4.0 * (min_value + 0.75 * val_range - min_value) / val_range
        b = 0.0
    return [r, g, b]