#!/usr/bin/env python
import roslib; roslib.load_manifest('smcimu')
import rospy
import re
import glob
import yaml

from sensor_msgs.msg import Imu

def smcimu():
    rospy.init_node('smcimu')
    pub = rospy.Publisher('~imu', Imu)

    # path to read the data from
    path = rospy.get_param('~path', None)
    if path == None:
        path = glob.iglob('/sys/devices/platform/applesmc.*/position').next()

    # path to read the calibration from
    calibrationPath = rospy.get_param('~calibration_file', None)
    if calibrationPath == None:
        calibrationPath = roslib.packages.resource_file('smcimu', 'info', 'calibration.yaml')

    # read calibration
    calib = None
    with open(calibrationPath, 'r') as f:
        calib = yaml.load(f.read())

    targetSamples = int(rospy.get_param('~samples', 5))
    targetRate = float(rospy.get_param('~rate', 30))
    rate = rospy.Rate(targetRate * targetSamples)

    # parse '(x,y,z)'
    parsePosition = re.compile('\((-?\d+),(-?\d+),(-?\d+)\)')

    while not rospy.is_shutdown():
        actualSamples = 0
        data = [0, 0, 0]
        # collect samples
        for i in range(targetSamples):
            try:
                with open(path) as f:
                    match = parsePosition.match(f.read())
                    data[0] += int(match.group(1))
                    data[1] += int(match.group(2))
                    data[2] += int(match.group(3))
                    actualSamples += 1
            except Exception as e:
                # this seems to happen quite a bit
                rospy.logwarn(rospy.get_name() + ': Error reading data: %s', str(e))
            rate.sleep()

        if actualSamples != 0:
            ix, iy, iz = [a / float(actualSamples) for a in data]
            iw = 1.0
            # transform the data
            x = ix * calib[0] + iy * calib[4] + iz * calib[8] + iw * calib[12]
            y = ix * calib[1] + iy * calib[5] + iz * calib[9] + iw * calib[13]
            z = ix * calib[2] + iy * calib[6] + iz * calib[10] + iw * calib[14]
            w = ix * calib[3] + iy * calib[7] + iz * calib[11] + iw * calib[15]
            x, y, z = [a / w for a in (x, y, z)]
            #w = 1.0
            # send out the Imu message
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.linear_acceleration.x = x
            imu.linear_acceleration.y = y
            imu.linear_acceleration.z = z
            # I don't know the covariance
            # these values should be transformed by the calibration matrix
            # I just copied how kinect_aux does this
            # the sensor in my MBP seems more accurate than this
            imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] = imu.linear_acceleration_covariance[8] = 0.01
            # -1 in covariance[0] means we don't support that data
            imu.angular_velocity_covariance[0] = -1
            imu.orientation_covariance[0] = -1
            pub.publish(imu)
        else:
            rospy.logerr(rospy.get_name() + ' gathered 0 samples', data)

if __name__ == '__main__':
    try:
        smcimu()
    except rospy.ROSInterruptException: pass
