# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
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

import math
import numpy as np
import rospy
from cyber_msgs.msg import GPGGA_MSG, Heading, GPTRA_MSG
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


class RosNMEADriver(object):
    def __init__(self):
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.fix_raw_pub = rospy.Publisher('raw_data', GPGGA_MSG, queue_size=1)
        self.heading_pub = rospy.Publisher('heading', Heading, queue_size=1)
        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.use_TRA = rospy.get_param('~useTRA', False)

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        #if not check_nmea_checksum(nmea_string):
            #rospy.logwarn("Received a sentence with an invalid checksum. " +
            #              "Sentence was: %s" % repr(nmea_string))
            #return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentece was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        
        current_raw_data = GPGGA_MSG()
        current_raw_data.header.stamp = current_time
        current_raw_data.header.frame_id = frame_id
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        current_heading = Heading()
        current_heading.header.frame_id = frame_id
        current_heading.header.stamp = current_time
        current_heading.status.status = 0
        current_attitude = GPTRA_MSG()
        current_attitude.header.stamp = current_time
        current_attitude.header.frame_id = frame_id
        current_vel = TwistStamped()
        current_vel.header.stamp = current_time
        current_vel.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC:
            if 'GPGGA' in parsed_sentence:
                data = parsed_sentence['GPGGA']
                gps_qual = data['fix_type']
                if gps_qual == 0:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX
                elif gps_qual == 1:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                elif gps_qual == 2:
                    current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
                elif gps_qual in (4, 5):
                    current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                current_fix.header.stamp = current_time

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                hdop = data['hdop']
                current_fix.position_covariance[0] = hdop ** 2
                current_fix.position_covariance[4] = hdop ** 2
                current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                # Altitude is above ellipsoid, so adjust for mean-sea-level
                altitude = data['altitude'] + 11 # data['mean_sea_level']
                current_fix.altitude = altitude
                current_raw_data.hdop = data['hdop']
                current_raw_data.num_satellites = data['num_satellites']
                current_raw_data.status = data['fix_type']
                self.fix_raw_pub.publish(current_raw_data)
                self.fix_pub.publish(current_fix)

            if 'PASHR' in parsed_sentence:
                data = parsed_sentence['PASHR']
                current_heading.data = data['heading']/180.0*np.pi
		current_heading.data = - current_heading.data # due to different installation methods. 
                if current_heading.data >= np.pi:
                    current_heading.data -= 2*np.pi
                if current_heading.data < -np.pi:
                    current_heading.data += 2*np.pi
                current_heading.std_dev = 0.1
                self.heading_pub.publish(current_heading)
            if 'HEADINGA' in parsed_sentence:
                data = parsed_sentence['HEADINGA']
                heading_sol_stat = data['sol_stat']
                heading_pos_type = data['pos_type']
                current_heading.status.status = current_fix.status.status
                if heading_sol_stat!='SOL_COMPUTED' or heading_pos_type=='NONE':
                    current_heading.status.status = -1
                current_heading.data = -data['heading']/180.0*np.pi + np.pi/2 - 0.55/180.0*np.pi#+ 0.17 * np.pi#plus the delta angle
                if current_heading.data >= np.pi:
                    current_heading.data -= 2*np.pi
                if current_heading.data < -np.pi:
                    current_heading.data += 2*np.pi
                current_heading.std_dev = data['hdg_std_dev']/180.0*np.pi
                self.heading_pub.publish(current_heading)
            else:
		        if 'GPTRA' in parsed_sentence:
		            data = parsed_sentence['GPTRA']
		            current_attitude.heading = -data['heading']/180.0*np.pi+np.pi/2 - 0.55/180.0*np.pi
		            if current_attitude.heading >= np.pi:
		                current_attitude.heading -= 2*np.pi
		            if current_attitude.heading < -np.pi:
		                current_attitude.heading += 2*np.pi
		            current_attitude.pitch = data['pitch']/180.0*np.pi
		            current_attitude.roll = data['roll']/180.0*np.pi
		            current_attitude.QF = data['QF']
		            current_attitude.sat_num = data['num_satellites'] 
		            current_heading.data = current_attitude.heading
		            current_heading.status.status = current_fix.status.status
		            current_heading.std_dev = np.nan
		            self.heading_pub.publish(current_heading)
                
        elif 'GPRMC' in parsed_sentence:
            data = parsed_sentence['GPRMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)
        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            """Add the TF prefix"""
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
