#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from utils.PCA9685 import PCA9685
from threading import Timer

PULSE_RANGE = 1000
PULSE_MAX = 2500
PULSE_MIN = 500


class FaceFollower:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)

        self.pulse = [1500.0, 1500.0]
        self.off = [0.0, 0.0]
        self.dir = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.timerDontSee = None

        self.move(self.pulse)

        rospy.init_node('face_follower', anonymous=True)

        rospy.Subscriber("face_pos", Point, self.cb_see_face)

        self.servo_pulses = rospy.Publisher(
            "servo_pulses", Point, queue_size=1)

        while not rospy.is_shutdown():
            self.update_servo(0)
            self.update_servo(1)

            servo_pulse = Point()
            servo_pulse.x = self.pulse[0]
            servo_pulse.y = self.pulse[1]
            self.servo_pulses.publish(servo_pulse)

            rospy.sleep(0.01)

    def update_servo(self, ax):
        target = self.dir[ax] * 3
        self.speed[ax] += (target - self.speed[ax]) / 20

        if abs(self.speed[ax]) > 0.01:
            self.is_moving = True
            self.pulse[ax] += self.speed[ax]
            self.pulse[ax] = max(PULSE_MIN, min(PULSE_MAX, self.pulse[ax]))
            self.pwm.setServoPulse(ax, self.pulse[ax])
        else:
            # pass
            # print('stop')
            self.pwm.stop(ax)


    def move(self, (px, py)):
        self.pwm.setServoPulse(0, px)
        self.pwm.setServoPulse(1, py)
        Timer(0.5, lambda: self.pwm.stop(0)).start()
        Timer(0.5, lambda: self.pwm.stop(1)).start()

    def reset_directions(self):
        print('reset directions')
        self.dir[0] = 0
        self.dir[1] = 0


    def cb_see_face(self, data):
        scale = 12
        if data.x > 0.23:
            self.dir[0] = -1
        elif data.x < -0.23:
            self.dir[0] = 1
        else:
            self.dir[0] = 0

        if data.y > 0.23:
            self.dir[1] = 1
        elif data.y < -0.23:
            self.dir[1] = -1
        else:
            self.dir[1] = 0
        
        if self.timerDontSee:
            self.timerDontSee.cancel()
        self.timerDontSee = Timer(0.5, self.reset_directions)
        self.timerDontSee.start()
        # self.pulse_x = self.pulse_x + data.x * scale
        # self.pulse_y = self.pulse_y + data.y * scale
        # self.move(self.pulse_x, self.pulse_y)


if __name__ == '__main__':
    ff = FaceFollower()
    rospy.spin()
