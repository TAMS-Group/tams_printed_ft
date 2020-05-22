This directory holds example Python ROS nodes for the printed-ft 
sensor prototypes:


* pushing-box.py - bare-bones ROS driver for the 2-DOF sensor.

  Reads incoming data from the specified serial device and baud-rate,
  then re-publishes as raw data. As a custom message would offer little
  benefit, and std_msgs has no 'easy' vector type, we simply hijack a
  sensor_msgs.msg.Joy() and fill its axes array with the measured force
  values and also the sequence number.


* block-ft.py - full six-axis F/T sensor ROS driver.

  This Python node offers the basic functionality for a six-axis
  F/T sensor, using linear calibration and reset-bias service.
  The node opens a serial communication channel using the specified
  device (e.g. /dev/ttyUSB0) and baudrate (e.g. 115200, ignored for USB)
  and then tries to parse the incoming data. For simplicity, we use
  plain-text format with decimal formatted numbers and linebreak as
  the separator; so that split() can be used.

  Incoming data packets are assumed to contain a start token 'D',
  the packet sequence number, an integer error-mask, followed by
  eight force values corresponding to the A/D readings for the
  eight optical sensors. 

  Invalid data packets are caught and printed to the terminal console
  for debugging, but are otherwise ignoread.

  Valid raw Raw is again re-published for debugging and plotting as
  a sensor_msgs.msg.Joy() message. Warning messages are generated if
  any of the single-channel readings are outside of the lower .. upper
  interval calculated from the calibration parameters.

  Afterwards, linear calibration is applied to the sensor readings
  and published as a geometry_msgs.msg.WrenchStamped() message using
  the sensor frame ("printed_ft_sensor_link"). The required calibration
  matrix is read only once at node startup from the parameter server;
  the corresponding launch files would typically upload a YAML file
  with all calibration parameters.

  In addition, the node starts a "set_zero_bias" service, that can
  be called at runtime to cancel the readings due to the mounted tool
  (if any) and due to drift.

  However, please note that the "quasi-linear" region of the optical
  F/T sensor is smaller than for typical strain-gage sensors, due to
  both the larger deflections needed and due to optical non-linearity.
  Use with care. If in doubt, it is better to re-calibrate the sensor
  in its typical orientation and with payload mounted, instead of 
  calibrating in a different position and relying on set_zero_bias() only.


* spacemouse_9904.py - six-axis F/T sensor driver.

  Both variants of the spacemouse sensor share a similar mechanical structure
  based on six optical sensors in a 60° layout, with fork-type interrupters
  for one prototype and reflex-type sensors for the "9904" variant.
 
  Full force/torque decoupling (Fx,Fy,Fz, Mx,My,Mz) is still possible, but 
  without the redundancy and extra robustness provided by the eight-sensor 
  layouts. While the fork-type spacemouse is quite sensitive, care must be taken 
  to avoid overload conditions, and to define adequate under/over-load limits, 
  as the useable (linear) deflection range of the interrupters is quite small.
  values outside the linear region will result in large measurment errors.

  Again, incoming serial data is parsed and raw-values are re-published as
  a Joy() message for simplicity. However, the spacemouse sensors can actually
  be used as (table-top or mobile) joystick devices with full 6-axis output.


* screwdriver_v3.py - custom driver for multi-modal wireless device.

  This code shares the same ideas already demonstrated for the other sensors.
  However, communication is now UDP/IP (WiFi) for wireless operation. 

  Output data from the device also includes IMU data (linear acceleration 
  and angular velocity: 6-DOF) in addition to the data from the optical 
  proximity-type sensors of both the tool-mounting part (5-DOF) and the 
  grasping-handle (2x6 DOF).




