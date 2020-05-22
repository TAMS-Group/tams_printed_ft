#! /usr/bin/env python 
#
# block_ft.py - "rectangular octo optocoupler f/t sensor"
#
# Note: design now defauilts to 230400 baud, so please
# set _baudrate:=230400 on the command line / launch file.
# 
# Basic Python ROS node to receive timestamps/sequence numbers
# and analog values (forces) sent by an Arduino (nano/mini-pro)
# via USB (serial), then publish as ROS Wrench message.
# Also publishes raw values as a sensor_msgs.Joy message.
#
# 2020.03.05 - copied (from big_ft.py)
# ...
# 2019.12.25 - created (copied from instrumented_screwdriver)
#
# (c) 2019, 2020 fnh, hendrich@informatik.uni-hamburg.de
#

import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import std_srvs.srv
# from sensor_msgs.msg import Imu

import collections
import numpy as np

# import socket
import serial
import threading   # we need a mutex...


mutex = threading.Lock()

wrench = geometry_msgs.msg.WrenchStamped()
joy = sensor_msgs.msg.Joy()
verbose = 0
have_calibration = True

# this node is currently expecting the Arduino nano with
# 8-channel ADC and 10 bits resolution. The node needs
# the "useful" ADC channel ranges to detect and handle
# overload conditions:
#
overload_lower   = [  50,  50,  50,  50,  50,  50, 50, 50 ]
overload_upper   = [ 950, 950, 950, 950, 950, 950, 950, 950 ]
clamped_data     = [ 0,0,0,0,0,0,0,0]
channel_offsets  = [ 0,0,0,0,0,0,0,0]
adc_range        = 1024;

calibration_fx = []
calibration_fy = []
calibration_fz = []
calibration_tx = []
calibration_ty = []
calibration_tz = []


def clamp( value, lower, upper):
    if (value >= upper): return upper
    if (value <= lower): return lower
    return value


# given the latest sensor readings, update the channel_offsets
# so that the output Wrench is all-zero.
# Note that the useful "working range" of this command is much smaller
# on our optical sensors than on traditional strain-gauges...
# 
def set_zero_bias( request ):
    global have_calibration
    global calibration_fx, calibration_fy, calibration_fz
    global calibration_tx, calibration_ty, calibration_tz
    global channel_offsets, clamped_data
    global wrench
    rospy.logerr( "set_zero_bias called..." )

    if not have_calibration:
        rospy.logerr( "set_zero_bias: No calibration loaded, cannot reset wrench!" )
        return
    #
    # We want: measuredWrench =!= 0
    # measuredWrench = coeffs[0] + sum( coeffs[i] * (clamped_data[i] - channel_offsets[i] )
    # 
    # coeffs[0] = -sum( coeffs[i] * (clamped_data[i] - channel_offsets[i] )
    #
    try: 
        mutex.acquire()
        rospy.loginfo( "got the mutex..." )

        fx = fy = fz = tx = ty = tz = 0
        for i in range( 8 ):
            print( "clamped_data", i, " ", clamped_data[i] )

            fx = fx + calibration_fx[i+1] * (clamped_data[i] - channel_offsets[i] )
            fy = fy + calibration_fy[i+1] * (clamped_data[i] - channel_offsets[i] )
            fz = fz + calibration_fz[i+1] * (clamped_data[i] - channel_offsets[i] )

            tx = tx + calibration_tx[i+1] * (clamped_data[i] - channel_offsets[i] )
            ty = ty + calibration_ty[i+1] * (clamped_data[i] - channel_offsets[i] )
            tz = tz + calibration_tz[i+1] * (clamped_data[i] - channel_offsets[i] )

        calibration_fx[0] = -fx
        calibration_fy[0] = -fy
        calibration_fz[0] = -fz

        calibration_tx[0] = -tx
        calibration_ty[0] = -ty
        calibration_tz[0] = -tz

        print( "set_zero_bias: new calibration_*[0] values: " )
        print( calibration_fx[0] )
        print( calibration_fy[0] )
        print( calibration_fz[0] )
        print( calibration_tx[0] )
        print( calibration_ty[0] )
        print( calibration_tz[0] )

    finally:
        mutex.release()
        rospy.loginfo( "release the mutex..." )
    
    empty = std_srvs.srv.Empty() # which is empty, and has no response...
    # print( dir( empty ))
    res = std_srvs.srv.EmptyResponse() # aha, extra weird types generates for us?
    # print( dir( res ))
    return res



def printed_ft():
    global verbose
    global wrench
    global joy
    global overload_lower # both threshold for detection and clamp for wrench calculation
    global overload_upper
    global channel_offsets
    global clamped_data
    global have_calibration
    global calibration_fx, calibration_fy, calibration_fz
    global calibration_tx, calibration_ty, calibration_tz
    print( "printed_ft init...\n" )

    # UDP Socket
    # ARDUINO_IP = "192.168.104.97";   # unused, hardcoded in INFFUL network setup
    # UDP_IP = "134.100.13.101"        # our own IP address
    # UDP_PORT = 11333;                # the port we use, must match Arduino settings

    # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    # sock.bind((UDP_IP, UDP_PORT))
    # print "got the UDP socket..."

    # ROS node stuff
    rospy.init_node( 'printed_ft', anonymous=False )
    wrench_publisher = rospy.Publisher( '~wrench', geometry_msgs.msg.WrenchStamped, queue_size=1)
    joy_publisher = rospy.Publisher( '~rawdata', sensor_msgs.msg.Joy, queue_size=1)
    set_zero_bias_server = rospy.Service( '~set_zero_bias', std_srvs.srv.Empty, set_zero_bias )
    rate = rospy.Rate( 300 )
    print( "got the ROS node and publishers..." )

    if rospy.has_param( '~verbose' ):
        verbose = rospy.get_param( '~verbose' )
        print( "verbose = ", verbose )


    # Note: on Teensy and the like, port and baudrate setup is automatic,
    # as communication is directly via USB. For communication with older
    # boards via FTDI adapters (or similar), you should specify the comm
    # parameters...
    # 
    if rospy.has_param( '~serial_port' ):
        port = rospy.get_param( '~serial_port' )
    else:
        port = '/dev/ttyUSB0'  # Arduino nano
    print( 'serial device is: ', port )

    if rospy.has_param( '~baudrate' ):
        baudrate = rospy.get_param( '~baudrate' )
    else:
        baudrate = 115200

    # ser = serial.Serial( port )  # open serial port, Teensy
    # ser = serial.Serial('/dev/ttyUSB0')  # open serial port, Arduino nano
    # Arduino mini-pro via Sparkfun FTDI adapter
    ser = serial.Serial( port, baudrate, parity=serial.PARITY_NONE )  # open serial port
    # 
    print( "got the serial port" )

    # rospy private names NOT working at all? At least had (massive) trouble
    # accessing the calibration parameters using ~/calibration/blablubb:
    #
    print( "rospy node name: ", rospy.get_name() )
    print( "rospy namespace: ", rospy.get_namespace() )
    #print( "resolving name: ~/calibration/bias:" )
    #print( rospy.resolve_name( '~/calibration/bias' ))
    #print( rospy.resolve_name( '~calibration/bias' ))

    # calibration ? bias linear0 .. linearN-1 quadratic00 .. quadraticN-1N-1 ...
    # 
    n_sensors = 8
    have_calibration = True
    if rospy.has_param( '~calibration/n_channels' ): # take either n_channels or n_sensors
       n_sensors = rospy.get_param( '~calibration/n_channels' )
       print( type( n_sensors ))
       rospy.loginfo( '~calibration/n_sensors: %s', str(n_sensors) )
    elif rospy.has_param( '~n_channels' ):
       n_sensors = rospy.get_param( '~n_channels' )
       print( type( n_sensors ))
       rospy.loginfo( '~calibration/n_sensors: %s', str(n_sensors) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/offsets' ):
       channel_offsets = rospy.get_param( '~calibration/offsets' )
       if type(channel_offsets[0]) == str:
           channel_offsets = np.fromstring( channel_offsets[0], dtype=float, count=-1, sep=' ' )
       rospy.loginfo( 'calibration/offsets: %s', str(channel_offsets) )
    else: have_calibration = False

    print( n_sensors )
    print( channel_offsets )

    if rospy.has_param( '~calibration/fx' ):
       calibration_fx = rospy.get_param( '~calibration/fx' )
       if type(calibration_fx[0]) == str:
            calibration_fx = np.fromstring( calibration_fx[0], dtype=float, count=-1, sep=' ' )
       rospy.loginfo( 'calibration/fx: %s', str(calibration_fx) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/fy' ):
       calibration_fy = rospy.get_param( '~calibration/fy' )
       rospy.loginfo( 'calibration/fy: %s', str(calibration_fy) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/fz' ):
       calibration_fz = rospy.get_param( '~calibration/fz' )
       rospy.loginfo( 'calibration/fz: %s', str(calibration_fz) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/tx' ):
       calibration_tx = rospy.get_param( '~calibration/tx' )
       rospy.loginfo( 'calibration/tx: %s', str(calibration_tx) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/ty' ):
       calibration_ty = rospy.get_param( '~calibration/ty' )
       rospy.loginfo( 'calibration/ty: %s', str(calibration_ty) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/tz' ):
       calibration_tz = rospy.get_param( '~calibration/tz' )
       rospy.loginfo( 'calibration/tz: %s', str(calibration_tz) )
    else: have_calibration = False

    if rospy.has_param( '~calibration/overload_lower' ): # optional, not required
       overload_lower = rospy.get_param( '~calibration/overload_lower' )
       rospy.loginfo( 'calibration/overload_lower: %s', str(overload_lower) )

    if rospy.has_param( '~calibration/overload_upper' ): # optional, not required
       overload_upper = rospy.get_param( '~calibration/overload_upper' )
       rospy.loginfo( 'calibration/overload_upper: %s', str(overload_upper) )

    iteration = 0
    while not rospy.is_shutdown():
        iteration = iteration + 1

        force0 = force1 = force2 = force3 = 0
        force4 = force5 = force6 = force7 = 0
        try:
             data = ser.readline()
             if (verbose > 1) or (iteration < 10) or ((iteration % 1000) == 0):
                 print "received message: ", data

             (str_D, seq_number, error_mask, force0, force1, force2, force3, force4, force5, force6, force7) = \
                 [t(s) for t,s in zip((str,int,int,  int,int,int,int, int,int,int,int), data.split()) ]

        except ValueError:
             print( "parsing input data failed, data='", data, "'" )
             continue

        except IndexError: # probably wrong formatted string...
             print "could not parse message/data:", data
             continue

        except KeyboardInterrupt:
             pass
 
        # check for overload 
        overload = 0
        if (force0 < overload_lower[0]) or (force0 > overload_upper[0]):
            rospy.logwarn_throttle( 1, "Force overload channel 0" )
            overload = 1
        if (force1 < overload_lower[1]) or (force1 > overload_upper[1]):
            rospy.logwarn_throttle( 1, "Force overload channel 1" )
            overload = 1
        if (force2 < overload_lower[2]) or (force2 > overload_upper[2]):
            rospy.logwarn_throttle( 1, "Force overload channel 2" )
            overload = 1
        if (force3 < overload_lower[3]) or (force3 > overload_upper[3]):
            rospy.logwarn_throttle( 1, "Force overload channel 3" )
            overload = 1
        if (force4 < overload_lower[4]) or (force4 > overload_upper[4]):
            rospy.logwarn_throttle( 1, "Force overload channel 4" )
            overload = 1
        if (force5 < overload_lower[5]) or (force5 > overload_upper[5]):
            rospy.logwarn_throttle( 1, "Force overload channel 5" )
            overload = 1
        if (force6 < overload_lower[6]) or (force6 > overload_upper[6]):
            rospy.logwarn_throttle( 1, "Force overload channel 6" )
            overload = 1
        if (force7 < overload_lower[7]) or (force7 > overload_upper[7]):
            rospy.logwarn_throttle( 1, "Force overload channel 7" )
            overload = 1

        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = "printed_ft_sensor_link"
        joy.axes = [] # no easy resize() in Python

        joy.axes.append( force0 )
        joy.axes.append( force1 )
        joy.axes.append( force2 )
        joy.axes.append( force3 )

        joy.axes.append( force4 )
        joy.axes.append( force5 )
        joy.axes.append( force6 )
        joy.axes.append( force7 )

        joy.axes.append( 43 ) # just a separator between forces and other stuff
        joy.axes.append( overload )
        joy.axes.append( seq_number )
        joy.axes.append( error_mask )

        joy_publisher.publish( joy )

        clamped_data = []
        for i in range( 8 ):
            tmp = clamp( joy.axes[i], overload_lower[i], overload_upper[i] )
            clamped_data.append( tmp )

        if (have_calibration):
            try: 
                mutex.acquire()
                wrench.header.stamp    = rospy.Time.now()
                wrench.header.frame_id = "printed_ft_sensor_link"

                wrench.wrench.force.x = calibration_fx[0]
                wrench.wrench.force.y = calibration_fy[0]
                wrench.wrench.force.z = calibration_fz[0]

                wrench.wrench.torque.x = calibration_tx[0]
                wrench.wrench.torque.y = calibration_ty[0]
                wrench.wrench.torque.z = calibration_tz[0]

                for i in range( n_sensors ): 
                    wrench.wrench.force.x += calibration_fx[1+i] * (clamped_data[i] - channel_offsets[i] )
                    wrench.wrench.force.y += calibration_fy[1+i] * (clamped_data[i] - channel_offsets[i] )
                    wrench.wrench.force.z += calibration_fz[1+i] * (clamped_data[i] - channel_offsets[i] )
    
                    wrench.wrench.torque.x += calibration_tx[1+i] * (clamped_data[i] - channel_offsets[i] )
                    wrench.wrench.torque.y += calibration_ty[1+i] * (clamped_data[i] - channel_offsets[i] )
                    wrench.wrench.torque.z += calibration_tz[1+i] * (clamped_data[i] - channel_offsets[i] )

            finally:
                mutex.release()

            # scale by adc_range
            wrench.wrench.force.x = wrench.wrench.force.x / adc_range
            wrench.wrench.force.y = wrench.wrench.force.y / adc_range
            wrench.wrench.force.z = wrench.wrench.force.z / adc_range
            wrench.wrench.torque.x = wrench.wrench.torque.x / adc_range
            wrench.wrench.torque.y = wrench.wrench.torque.y / adc_range
            wrench.wrench.torque.z = wrench.wrench.torque.z / adc_range

            wrench_publisher.publish( wrench ) 

        rate.sleep()

    print "printed_ft stopped." 
    


if __name__ == '__main__':
    try:
        printed_ft()
    except KeyboardInterrupt:
        print "Received control-c, stopping..." 
        exit( 0 )
    except rospy.ROSInterruptException:
        pass

