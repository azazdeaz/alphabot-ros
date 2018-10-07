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

        self.pulse_x = 1500.0
        self.pulse_y = 1500.0
        self.off_x = 0.0
        self.off_y = 0.0
        self.dir_x = 0.0
        self.dir_y = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.timerDontSee = None
        
        self.move(self.pulse_x, self.pulse_y)

        rospy.init_node('face_follower', anonymous=True)

        rospy.Subscriber("face_pos", Point, self.callback)

        self.servo_pulses = rospy.Publisher("servo_pulses", Point, queue_size=1)

        while not rospy.is_shutdown():
            target_x = self.dir_x * 3
            self.speed_x += (target_x - self.speed_x) / 20

            print(self.pulse_x, self.speed_x, target_x)
            if abs(self.speed_x) > 0.01:
                self.pulse_x += self.speed_x
                self.pulse_x = max(PULSE_MIN, min(PULSE_MAX, self.pulse_x))
                self.pwm.setServoPulse(0, self.pulse_x)
                servo_pulse = Point()
                servo_pulse.x = self.pulse_x
                self.servo_pulses.publish(servo_pulse)
            else:
                self.pwm.stop(0)
            
            rospy.sleep(0.01)



    def move(self, px, py):
        self.pwm.setServoPulse(0, px)
        self.pwm.setServoPulse(1, py)
        Timer(0.5, lambda: self.pwm.stop(0)).start()
        Timer(0.5, lambda: self.pwm.stop(1)).start()

    def reset_directions(self):
        print('reset directions')
        self.dir_x = 0
        self.dir_y = 0


    def callback(self, data):
        scale = 12
        if data.x > 0.3:
            self.dir_x = -1
        elif data.x < -0.3:
            self.dir_x = 1
        else:
            self.dir_x = 0
        
        if self.timerDontSee:
            self.timerDontSee.cancel()
        self.timerDontSee = Timer(0.1, self.reset_directions)
        self.timerDontSee.start()
        # self.pulse_x = self.pulse_x + data.x * scale
        # self.pulse_y = self.pulse_y + data.y * scale
        # self.move(self.pulse_x, self.pulse_y)


if __name__ == '__main__':
    ff = FaceFollower()
    rospy.spin()
