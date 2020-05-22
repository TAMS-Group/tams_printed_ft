#! /usr/bin/env python 
#
# pushing_ft.py 
#
# Basic Python ROS node to receive timestamps/sequence numbers
# and analog values (forces) sent by an Arduino (nano/mini-pro)
# via USB (serial), then publish as ROS sensor_msgs.Joy message.
#
# 2020.03.09 - updated (copied from bottle_ft.py)
#
# (c) 2019, 2020 fnh, hendrich@informatik.uni-hamburg.de
#

import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

import collections
import numpy as np

import serial


joy = sensor_msgs.msg.Joy()
verbose = 0

# using 10-bit ADCs, and the given LED current and pullup resistors,
# this is about the useful range. Anything lower or higher incurs
# nonlinear behaviour and consists an overload condition.
#
overload_lower = 100
overload_upper = 900


def pushing_ft():
    global joy
    global verbose
    print "pushing_ft init...\n" 


    # ROS node stuff
    rospy.init_node( 'pushing_ft', anonymous=False )
    joy_publisher = rospy.Publisher( '~rawdata', sensor_msgs.msg.Joy, queue_size=1)

    # Note: on Teensy and the like, port and baudrate setup is automatic,
    # as communication is directly via USB. For communication with older
    # boards via FTDI adapters (or similar), you should specify the comm
    # parameters...
    # 
    # ser = serial.Serial('/dev/ttyUSB0', 112500, parity=serial.PARITY_NONE )  # open serial port
    # ser = serial.Serial('/dev/ttyACM0')  # open serial port, Teensy
    port = '/dev/ttyUSB0'
    if rospy.has_param( '~port' ):
        port = rospy.get_param( '~port' )
        print( "Using serial port ", port )

    baudrate = 230400
    if rospy.has_param( '~baudrate' ):
        baudrate = rospy.get_param( '~baudrate' )
        print( "Using baudrate ", baudrate )

    ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE )  # open serial port, Arduino nano
    print "got the serial port"

    rate = rospy.Rate( 300 )
    print "got the ROS node and publishers..." 

    iteration = 0
    while not rospy.is_shutdown():
        iteration = iteration + 1
        # print "jojo", iteration

        force0 = force1 = force2 = force3 = 0
        force4 = force5 = force6 = force7 = 0
        error_mask = 0
        try:
             data = ser.readline()
             if (verbose > 1) or (iteration < 10) or ((iteration % 1000) == 999):
                  print "received message: ", data

             (str_D, seq_number, error_mask, force0, force1, force2, force3, force4, force5, force6, force7) = \
                 [t(s) for t,s in zip((str,int,int, int,int,int,int, int,int,int,int), data.split()) ]

        except ValueError:
             print "parsing input data failed:"
             print data
             continue

        except KeyboardInterrupt:
             pass
 
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = "pushing_ft_sensor_link"
        joy.axes = [] # no easy resize() in Python

        joy.axes.append( force0 )
        joy.axes.append( force1 )
        #joy.axes.append( force2 )
        #joy.axes.append( force3 )
        #
        #joy.axes.append( force4 )
        #joy.axes.append( force5 )
        #joy.axes.append( force6 )
        #joy.axes.append( force7 )

        joy.axes.append( seq_number )

        joy_publisher.publish( joy )
        rate.sleep()

    print "pushing_ft stopped." 
    


if __name__ == '__main__':
    try:
        pushing_ft()
    except KeyboardInterrupt:
        print "Received control-c, stopping..." 
        exit( 0 )
    except rospy.ROSInterruptException:
        pass

