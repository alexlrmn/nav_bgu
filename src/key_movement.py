#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import roslib
import datetime

import sys, select, termios, tty


class keyMoveImprove:

    msg = """
    To enable key movement: rostopic pub -1 /emergency_stop std_msgs/Bool -- False
    To disable key movement: rostopic pub -1 /emergency_stop std_msgs/Bool -- True

    Key movement will only work in the terminal you launched the node

    Control Your Komodo Movement!
    ---------------------------
    Moving around:
       q    w    e
       a    s    d
       m    ,    .
    NUM7/NUM9 : increase/decrease max speeds by 10%
    NUM8/NUM2 : increase/decrease only linear speed by 10%
    NUM4/NUM6 : increase/decrease only angular speed by 10%
    space key, k : force stop
    anything else : stop smoothly
    CTRL-C to quit
    """

    moveBindings = {
        'w': (1, 0),
        'e': (1, -1),
        'a': (0, 1),
        'd': (0, -1),
        'q': (1, 1),
        's': (-1, 0),
        '.': (-1, 1),
        'm': (-1, -1),
    }

    speedBindings = {
        '7': (1.1, 1.1),
        '9': (.9, .9),
        '8': (1.1, 1),
        '2': (.9, 1),
        '4': (1, 1.1),
        '6': (1, .9),
    }

    speed = 0.2
    turn = 0.5

    emerg_stop = False

    movement_forward = True

    settings = termios.tcgetattr(sys.stdin)

    def __init__(self):

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        #self.pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=5)
        self.stop_forward = rospy.Subscriber('/stop_movement_forward', Bool, self.stop_movement_forward_callback)
        self.emergency_stop = rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop_callback)
        self.movement()

    def movement(self):
        x = 0
        th = 0
        status = 0
        count = 0
        acc = 0.1
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        try:

            print self.msg

            while not self.emerg_stop:
                key = self.getKey()

                if key in self.moveBindings.keys():
                    if ('w' == key or 'q' == key or 'e' == key) and not self.movement_forward:
                        print 'Obstacle in front'
                        if x > 0:
                            x = 0
                    else:
                        x = self.moveBindings[key][0]
                        th = self.moveBindings[key][1]
                        count = 0
                        print "movement key pressed"
                elif key in self.speedBindings.keys():

                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    count = 0

                    print self.vels(self.speed, self.turn)
                    if 14 == status:
                        print self.msg
                    status = (status + 1) % 15
                elif key == ' ' or key == 'k':

                    x = 0
                    th = 0
                    control_speed = 0
                    control_turn = 0
                    print 'stop!'
                else:
                    count += 1
                    if count > 4:
                        x = 0
                        th = 0
                    if '\x03' == key:
                        break

                target_speed = self.speed * x
                target_turn = self.turn * th

                if target_speed > control_speed:
                    control_speed = min(target_speed, control_speed + 0.02)
                elif target_speed < control_speed:
                    control_speed = max(target_speed, control_speed - 0.02)
                else:
                    control_speed = target_speed

                if target_turn > control_turn:
                    control_turn = min(target_turn, control_turn + 0.1)
                elif target_turn < control_turn:
                    control_turn = max(target_turn, control_turn - 0.1)
                else:
                    control_turn = target_turn

                twist = Twist()
                twist.linear.x = control_speed; twist.linear.y = 0;  twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
                self.pub.publish(twist)



        except Exception as e:
            print e

        finally:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            self.pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self):

        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)

        if rlist:

            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def stop_movement_forward_callback(self, stop_movement_forward):
        if stop_movement_forward.data:
            self.movement_forward = False
        else:
            self.movement_forward = True;

    def emergency_stop_callback(self, emergency_stop):
        self.emerg_stop = emergency_stop.data
        if not self.emerg_stop:
            self.movement()
        else:
            self.stop_completely()

    def stop_completely(self):
        print 'Stop!'
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.pub.publish(twist)


def main(args):
    rospy.init_node('keyMoveImprove', anonymous=True)

    keyMoveImprove()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)

