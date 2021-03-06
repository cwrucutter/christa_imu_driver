#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, EJ Kreinar
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
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
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

import rospy
from sensor_msgs.msg import Imu

import serial, string, math, struct

crctable= [0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
           0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 ]

#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)

#Christa's provided Checksum function
#Calculate the CRC for supplied data (bytearray)
def CRC16(data):
    CRC = 0;
    for idx in range(len(data)):
        CRC = (CRC >> 4) ^ crctable[((CRC ^ (data[idx] & 0xF)) & 0xF)]
        CRC = (CRC >> 4) ^ crctable[((CRC ^ (data[idx] >> 4 )) & 0xF)]
    return CRC


class ChristaParser:
    """
        Parses a binary Christa imuSerial Message
    """
    def __init__(self):
        """ Initialize the NovatelParser """
        self.hdr_msgID   = 0;
        self.hdr_msgLen  = 0;

        self.HS_SERIAL_IMU_MSG = 255;
        self.HS_SERIAL_IMU_MSG_LEN = 18;

    def VerifyChecksum(self, data, CRC):
        """ Verify the Checksum, return bool """
        chk = struct.unpack('>H', CRC)
        checksum = CRC16(bytearray(data))
        return (chk[0] == checksum)
    
    def ParseHeader(self, data):
        """ Read the Header, return bool if we have the correct messageID and Length """
        header = struct.unpack('<BB',data);
        self.hdr_msgID  = header[0]
        self.hdr_msgLen = header[1]
        return True

    def ParseHsSerialImu(self, data, imuMsg):
        """ Parse a High Speed Serial message, populate imuMsg
            data: string
            imuMsg: ROS message Imu
        """
        hs_serial = struct.unpack('>hhhhhhIBB',msg)
        gyroX, gyroY, gyroZ, accelX, accelY, accelZ, \
        timeSincePPS, PPSCount, Sequence  = hs_serial
        rospy.loginfo(hs_serial)

        # conversions from the christa datasheet:
        #    http://www.cloudcaptech.com/Download/Sensors/Crista%20IMU/Docs/Crista%20IMU%20Comms%20Spec.pdf

        # Populate the Angular Velocity
        imuMsg.angular_velocity.x = gyroX * ( 2.0 * 300 / 65536) * (math.pi / 180)
        imuMsg.angular_velocity.y = gyroY * ( 2.0 * 300 / 65536) * (math.pi / 180)
        imuMsg.angular_velocity.z = gyroZ * ( 2.0 * 300 / 65536) * (math.pi / 180)
        #TODO: Populate the covariance

        # Populate the Linear Acceleration
        imuMsg.linear_acceleration.x = accelX * ( 19.61 * 10 / 65536)
        imuMsg.linear_acceleration.y = accelY * ( 19.61 * 10 / 65536)
        imuMsg.linear_acceleration.z = accelZ * ( 19.61 * 10 / 65536)
        #TODO: Populate the covariance

        #TODO: Populate the orientation (But christa IMU doesnt have a magnetometer :( )

        return True


if __name__ == "__main__":
    #ROS init
    rospy.init_node('christa_imu_driver')
    imuPub = rospy.Publisher('imu/data',Imu,queue_size=1)
    #Init Imu port
    imuPort = rospy.get_param('~port','/dev/ttyUSB0')
    imuRate = rospy.get_param('~baud',115200)
#frame_id = rospy.get_param('~frame_id','imu') # maybe publish a frame or something so we can convert the acceleration values?
#if frame_id[0] != "/":
#frame_id = addTFPrefix(frame_id)

    imuMsg = Imu()
    imuMsg.header.frame_id = "base_imu"

    parser = ChristaParser()
    
    try:
        #Create a Serial object. We will use the funcitons .read() and .close()
        imuSerial = serial.Serial(port=imuPort,baudrate=imuRate,timeout=.01)
        #Read in GPS data
        sync0 = '\x00'; sync1 = '\x00';# sync2 = '\x00';
        while not rospy.is_shutdown():
            # READ UNTIL SYNC
            data  = imuSerial.read(1)
            sync1 = sync0;
            sync0 = data;
            sync  = sync1+sync0;
            match = '\x55\xAA'
            if sync != match:
                continue
            else:
                rospy.loginfo("Beginning new message")

            # READ HEADER
            # parser.hdr_msgID and parser.hdr_msgLen get set during the 
            # parser.ParseHeader function call.
            header = imuSerial.read(2)
            if (not parser.ParseHeader(header)):
                rospy.logwarn("Packet Failed: Header could not be parsed")
                continue
            
            # READ MESSAGE
            msg = imuSerial.read(parser.hdr_msgLen)
            if (len(msg) != parser.hdr_msgLen):
                rospy.loginfo("Packet Failed: Message length unexpected")
                continue
            
            # READ CRC
            chk = imuSerial.read(2)
            if (not parser.VerifyChecksum(sync+header+msg,chk)):    
                rospy.logwarn("Packet Failed: CRC Did not Match")
                continue
            
            # PARSE MESSAGE
            timeNow = rospy.get_rostime()
            if parser.hdr_msgID == parser.HS_SERIAL_IMU_MSG:
                #HS_SERIAL_IMU_MSG message
                imuMsg.header.stamp = timeNow
                parser.ParseHsSerialImu(msg,imuMsg)
            
                # Publish imuMsg
                imuPub.publish(imuMsg)
            else:
                rospy.logwarn("Christa message ID not recognized. Supported messages: HS_SERIAL_IMU_MSG")
            
    except rospy.ROSInterruptException:
        imuSerial.close() #Close GPS serial port
