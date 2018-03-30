#!/usr/bin/env python
from lsm6ds0_msg_pb2 import lsm6ds0_msg
from sensor_msgs.msg import Imu
import serial
import struct
import rospy
import tf

def open_port():
    rospy.init_node('imu_publisher')
    port_name = "/dev/ttyACM0"
    portIsClosed = True

    while portIsClosed:
        try:
            serial_port = serial.Serial(port=port_name, baudrate=19200)
            portIsClosed = False
            rospy.loginfo('Port name = ' + port_name)
            return True, serial_port
        except:
            portIsClosed = True
            rospy.logerr('Cannot open a port. Port name could be wrong.')
            rospy.sleep(0.5)

def imu_publisher(serial_port):
    pub = rospy.Publisher('/Imu', Imu, queue_size=100)
    rospy.init_node('imu_publisher')

    msg_receive = lsm6ds0_msg()
    msg_send = Imu()
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        sync_byte = serial_port.read(1)
        if sync_byte == '\xff':
            try:
                size_byte = struct.unpack("B", serial_port.read(1))[0]
                s = serial_port.read(size_byte)
                msg_receive.ParseFromString(s)
                rospy.loginfo(msg_receive)
                try:
                    br.sendTransform((0,0,0), (msg_receive.q0, msg_receive.q1, msg_receive.q2, msg_receive.q3),
                                     rospy.Time.now(),'imu','base_link')
                except:
                    rospy.logerr('Cannot send the tf msg')
                    pass


            except:
                rospy.logerr('Cannot read data from uC')
                pass

        msg_send.linear_acceleration.x = msg_receive.acc_x
        msg_send.linear_acceleration.y = msg_receive.acc_y
        msg_send.linear_acceleration.z = msg_receive.acc_z

        msg_send.angular_velocity.x = msg_receive.gyro_x
        msg_send.angular_velocity.y = msg_receive.gyro_y
        msg_send.angular_velocity.z = msg_receive.gyro_z

        msg_send.orientation.x = msg_receive.q0
        msg_send.orientation.y = msg_receive.q1
        msg_send.orientation.z = msg_receive.q2
        msg_send.orientation.w = msg_receive.q3

        msg_send.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        msg_send.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        msg_send.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        pub.publish(msg_send)

    if serial_port.is_open == True:
        serial_port.close()

if __name__ == '__main__':
    try:
        status, serial_port = open_port()
        if status == True:
            imu_publisher(serial_port)
        else:
            rospy.logerr('Cannot publish message from IMU')
    except rospy.ROSInterruptException:
        pass
