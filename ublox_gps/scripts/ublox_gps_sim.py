#!/usr/bin/python

import rospy, math
import numpy as np
from geographiclib.geodesic import Geodesic
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavPVT, NavVELNED, NavRELPOSNED
import tf.transformations as TR

class Spoofer(object):
    def __init__(self):
        self.pubPVT = False
        self.pubRPN = False
        self.pubVEL = False
        self.posNED =     None
        self.velNED =     None
        self.posBaseNED = None
        self.refLLA = [rospy.get_param('~reference_latitude'), rospy.get_param('~reference_longitude'), rospy.get_param('~ground_altitude')]
        self.frame_id = rospy.get_param('~frame_id')

        self.sample_time = 1.0

        self.abs_N_stdev = rospy.get_param('~abs_north_stdev')
        self.abs_E_stdev = rospy.get_param('~abs_east_stdev')
        self.abs_A_stdev = rospy.get_param('~abs_alt_stdev')
        self.abs_N_k     = rospy.get_param('~abs_north_k_GPS')
        self.abs_E_k     = rospy.get_param('~abs_east_k_GPS')
        self.abs_A_k     = rospy.get_param('~abs_alt_k_GPS')

        self.abs_N_error = 0.0
        self.abs_E_error = 0.0
        self.abs_A_error = 0.0

        self.abs_Nv_stdev = rospy.get_param('~abs_north_vel_stdev')
        self.abs_Ev_stdev = rospy.get_param('~abs_east_vel_stdev')
        self.abs_Dv_stdev = rospy.get_param('~abs_down_vel_stdev')

        self.rel_N_stdev = rospy.get_param('~rel_north_stdev')
        self.rel_E_stdev = rospy.get_param('~rel_east_stdev')
        self.rel_D_stdev = rospy.get_param('~rel_down_stdev')

        param_dict = rospy.get_param('~publish/nav')
        if 'pvt' in param_dict:
            self.pubPVT = param_dict['pvt']
        if 'relposned' in param_dict:
            self.pubRPN = param_dict['relposned']
        if 'velned' in param_dict:
            self.pubVEL = param_dict['velned']

        self.possub = rospy.Subscriber("truth", Odometry, self.callback, queue_size=1)

        if self.frame_id == 'rover':
            self.otherSub = rospy.Subscriber("base_truth", Odometry,
                self.otherCallback, queue_size=1)

        if self.pubPVT:
            self.pvt_pub = rospy.Publisher(self.frame_id + "/navpvt", NavPVT, queue_size=1)
        if self.pubRPN and self.frame_id == 'rover':
            self.rpn_pub = rospy.Publisher(self.frame_id + "/navrelposned", NavRELPOSNED, queue_size=1)
        if self.pubVEL:
            self.vel_pub = rospy.Publisher(self.frame_id + "/navvelned", NavVELNED, queue_size=1)

        if self.pubPVT or self.pubRPN:
            self.slow_update_timer = rospy.Timer(rospy.Duration(1.0), self.slow_update)
        if self.pubVEL:
            self.fast_update_timer = rospy.Timer(rospy.Duration(1.0/2.0), self.fast_update)

    def otherCallback(self, msg):
        # Truth frame is NED
        N = msg.pose.pose.position.x
        E = msg.pose.pose.position.y
        D = msg.pose.pose.position.z
        self.posBaseNED = [N, E, D]

    def callback(self, msg):
        # Truth frame is NED; velocity needs to be re-expressed from body to NED frame
        N = msg.pose.pose.position.x
        E = msg.pose.pose.position.y
        D = msg.pose.pose.position.z
        self.posNED = [N, E, D]

        q_UAV_NED = msg.pose.pose.orientation
        R_UAV_NED = TR.quaternion_matrix((q_UAV_NED.x,q_UAV_NED.y,q_UAV_NED.z,q_UAV_NED.w))[0:3,0:3]

        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        w = msg.twist.twist.linear.z
        vel_UAV = np.array([u, v, w])
        vel_NED = np.dot(R_UAV_NED, vel_UAV)

        self.velNED = [vel_NED[0], vel_NED[1], vel_NED[2]]

    def slow_update(self, event):
        if self.pubPVT and (not self.posNED is None) and (not self.velNED is None):
            # Add error/noise to measurements
            self.abs_N_error = np.exp(-1.0*self.abs_N_k*self.sample_time)*self.abs_N_error + np.random.normal(0.0, self.abs_N_stdev)
            self.abs_E_error = np.exp(-1.0*self.abs_E_k*self.sample_time)*self.abs_E_error + np.random.normal(0.0, self.abs_E_stdev)
            self.abs_A_error = np.exp(-1.0*self.abs_A_k*self.sample_time)*self.abs_A_error + np.random.normal(0.0, self.abs_A_stdev)
            N =  self.posNED[0] + self.abs_N_error
            E =  self.posNED[1] + self.abs_E_error
            A = -self.posNED[2] + self.abs_A_error
            lat_deg, lon_deg, alt_m = self.nea_to_gps(N, E, A)

            vN = self.velNED[0] + np.random.normal(0.0, self.abs_Nv_stdev)
            vE = self.velNED[1] + np.random.normal(0.0, self.abs_Ev_stdev)
            vD = self.velNED[2] + np.random.normal(0.0, self.abs_Dv_stdev)

            # Publish
            pvtmsg = NavPVT()
            pvtmsg.valid = NavPVT.VALID_FULLY_RESOLVED
            pvtmsg.fixType = NavPVT.FIX_TYPE_3D
            pvtmsg.flags = NavPVT.FLAGS_GNSS_FIX_OK
            pvtmsg.numSV = int(7)
            pvtmsg.lon = int(lon_deg * 1.0e7)
            pvtmsg.lat = int(lat_deg * 1.0e7)
            pvtmsg.height = int(alt_m * 1.0e3)
            pvtmsg.hMSL = pvtmsg.height
            pvtmsg.hAcc = int(np.sqrt(self.abs_N_stdev**2+self.abs_E_stdev**2) * 1.0e3)
            pvtmsg.vAcc = int(self.abs_A_stdev * 1.0e3)
            pvtmsg.velN = int(vN * 1.0e3)
            pvtmsg.velE = int(vE * 1.0e3)
            pvtmsg.velD = int(vD * 1.0e3)
            pvtmsg.gSpeed = int(np.sqrt(vN**2+vE**2) * 1.0e3)
            pvtmsg.heading = int(np.arctan2(vE, vN) * 1.0e5)
            pvtmsg.sAcc = int(np.sqrt(self.abs_Nv_stdev**2+self.abs_Ev_stdev**2) * 1.0e3)
            pvtmsg.headAcc = 10000
            self.pvt_pub.publish(pvtmsg)

        if self.pubRPN and (not self.posNED is None) and (not self.posBaseNED is None):
            # Add error/noise to measurements
            rN = self.posNED[0] - self.posBaseNED[0] + np.random.normal(0.0, self.rel_N_stdev)
            rE = self.posNED[1] - self.posBaseNED[1] + np.random.normal(0.0, self.rel_E_stdev)
            rD = self.posNED[2] - self.posBaseNED[2] + np.random.normal(0.0, self.rel_D_stdev)

            # Decompose into [cm] and [0.1mm] precision components
            rPN = int(rN * 1.0e2)
            rHPN = int(1.0e4 * (rN - 1.0e-2 * rPN))
            rPE = int(rE * 1.0e2)
            rHPE = int(1.0e4 * (rE - 1.0e-2 * rPE))
            rPD = int(rD * 1.0e2)
            rHPD = int(1.0e4 * (rD - 1.0e-2 * rPD))

            # Publish
            rpnmsg = NavRELPOSNED()
            rpnmsg.relPosN = rPN
            rpnmsg.relPosHPN = rHPN
            rpnmsg.relPosE = rPE
            rpnmsg.relPosHPE = rHPE
            rpnmsg.relPosD = rPD
            rpnmsg.relPosHPD = rHPD
            rpnmsg.accN = int(self.rel_N_stdev * 1.0e4)
            rpnmsg.accE = int(self.rel_E_stdev * 1.0e4)
            rpnmsg.accD = int(self.rel_D_stdev * 1.0e4)
            self.rpn_pub.publish(rpnmsg)

    def fast_update(self, event):
        if self.pubVEL and (not self.velNED is None):
            # Add error/noise to measurements
            vN = self.velNED[0] + np.random.normal(0.0, self.abs_Nv_stdev)
            vE = self.velNED[1] + np.random.normal(0.0, self.abs_Ev_stdev)
            vD = self.velNED[2] + np.random.normal(0.0, self.abs_Dv_stdev)

            # Publish
            velmsg = NavVELNED()
            velmsg.velN = int(vN * 1.0e2)
            velmsg.velE = int(vE * 1.0e2)
            velmsg.velD = int(vD * 1.0e2)
            velmsg.sAcc = int(np.sqrt(self.abs_Nv_stdev**2+self.abs_Ev_stdev**2) * 1.0e2)
            self.vel_pub.publish(velmsg)

    def nea_to_gps(self, north, east, alt = 0):
        arc_distance = math.sqrt(north**2+east**2)
        azi_1 = math.degrees(math.atan2(east, north))
        diction = Geodesic.WGS84.Direct(self.refLLA[0], self.refLLA[1], azi_1, arc_distance)
        return diction['lat2'], diction['lon2'], alt - self.refLLA[2]

def main():
    '''Initialize and cleanup ros node'''
    rospy.init_node('ublox_gps_spoofer', anonymous=True)
    s = Spoofer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS ublox gps spoofer node."

if __name__ == '__main__':
    main()
