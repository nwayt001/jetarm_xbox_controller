#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 21 19:09:15 2016

@author: nicholas
"""

import os, struct, array
from fcntl import ioctl

# Simple Joystick
class SimpleJoystick(object):
    """
    Simple Joystick: reads values from an xbox 360 controller
    """
    
    def __init__(self):
        # should we continue polling
        self.polling = True
        self.debug_mode = False
        # We'll store the states here.
        self.axis_states = {}
        self.button_states = {}
        # Joystick threshold for converting analog stick values to discrete values
        self.stickTreshold = 0.6
        # These constants were borrowed from linux/input.h
        self.axis_names = {
            0x00 : 'x',
            0x01 : 'y',
            0x02 : 'z',
            0x03 : 'rx',
            0x04 : 'ry',
            0x05 : 'rz',
            0x06 : 'trottle',
            0x07 : 'rudder',
            0x08 : 'wheel',
            0x09 : 'gas',
            0x0a : 'brake',
            0x10 : 'hat0x',
            0x11 : 'hat0y',
            0x12 : 'hat1x',
            0x13 : 'hat1y',
            0x14 : 'hat2x',
            0x15 : 'hat2y',
            0x16 : 'hat3x',
            0x17 : 'hat3y',
            0x18 : 'pressure',
            0x19 : 'distance',
            0x1a : 'tilt_x',
            0x1b : 'tilt_y',
            0x1c : 'tool_width',
            0x20 : 'volume',
            0x28 : 'misc',
        }

        self.button_names = {
            0x120 : 'trigger',
            0x121 : 'thumb',
            0x122 : 'thumb2',
            0x123 : 'top',
            0x124 : 'top2',
            0x125 : 'pinkie',
            0x126 : 'base',
            0x127 : 'base2',
            0x128 : 'base3',
            0x129 : 'base4',
            0x12a : 'base5',
            0x12b : 'base6',
            0x12f : 'dead',
            0x130 : 'a',
            0x131 : 'b',
            0x132 : 'c',
            0x133 : 'x',
            0x134 : 'y',
            0x135 : 'z',
            0x136 : 'tl',
            0x137 : 'tr',
            0x138 : 'tl2',
            0x139 : 'tr2',
            0x13a : 'select',
            0x13b : 'start',
            0x13c : 'mode',
            0x13d : 'thumbl',
            0x13e : 'thumbr',
        
            0x220 : 'dpad_up',
            0x221 : 'dpad_down',
            0x222 : 'dpad_left',
            0x223 : 'dpad_right',
        
            # XBox 360 controller uses these codes.
            0x2c0 : 'dpad_left',
            0x2c1 : 'dpad_right',
            0x2c2 : 'dpad_up',
            0x2c3 : 'dpad_down',
        }
        
        self.axis_map = []
        self.button_map = []
        
        # Open the joystick device.
        self.fn = '/dev/input/js0'
        print('Opening %s...' % self.fn)
        self.jsdev = open(self.fn, 'rb')
        
        # Get the device name.
        #buf = bytearray(63)
        buf = array.array('b', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tostring()
        print('Device name: %s' % js_name)
        
        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]
        
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]
        
        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP
        
        for axis in buf[:num_axes]:
            self.axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(self.axis_name)
            self.axis_states[self.axis_name] = 0.0
        
        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP
        
        for btn in buf[:num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0
        
        #print '%d axes found: %s' % (num_axes, ', '.join(self.axis_map))
        #print '%d buttons found: %s' % (num_buttons, ', '.join(self.button_map))
        
    def poll(self):
        # Main event loop to poll for joystick state changes
        while self.polling:
            evbuf = self.jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)
        
                if type & 0x01:
                    button = self.button_map[number]
                    if button:
                        self.button_states[button] = value
                        if(self.debug_mode):
                            if value:
                                print( "%s pressed" % (button))
                            else:
                                print( "%s released" % (button))
        
                if type & 0x02:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        if(self.debug_mode):
#                            if(axis!='x' and axis !='y'):
#                                print "%s: %.3f" % (axis, fvalue)
                            if(axis=='x'):
                                print(fvalue)
                        
    def get_state(self):
        # get state of xbox controller
        evbuf = self.jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)
    
            if type & 0x01:
                button = self.button_map[number]
                if button:
                    self.button_states[button] = value
                    if(self.debug_mode):
                        if value:
                            print("%s pressed" % (button))
                        else:
                            print("%s released" % (button))
    
            if type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    self.axis_states[axis] = fvalue
                    if(self.debug_mode):
#                            if(axis!='x' and axis !='y'):
#                                print "%s: %.3f" % (axis, fvalue)
                        if(axis=='x'):
                            print(fvalue)
        
if __name__ == '__main__':
    # Create a new Joystick 
    js = SimpleJoystick()
    js.debug_mode = True
    
    # Start polling the joystick
    js.poll()


        
        
        