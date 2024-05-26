import rospy
from geometry_msgs.msg import Twist

def getKey():
    key = input("Introdu o comandă (i/j/k/l/u/o/m/,/.): ")
    return key

def sendTwist(pub, x, y, z, th):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = th
    pub.publish(twist)

def teleopKeyboard():
    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    while (1):
        key = getKey()

        if key == 'i':
            sendTwist(pub, 1, 0, 0, 0)  # Înainte
        elif key == 'j':
            sendTwist(pub, 0, 1, 0, 0)  # Stânga
        elif key == 'k':
            sendTwist(pub, -1, 0, 0, 0)  # Înapoi
        elif key == 'l':
            sendTwist(pub, 0, -1, 0, 0)  # Dreapta
        elif key == 'u':
            sendTwist(pub, 1, 0, 0, 1)  # Înainte și rotație la stânga
        elif key == 'o':
            sendTwist(pub, 1, 0, 0, -1)  # Înainte și rotație la dreapta
        elif key == 'm':
            sendTwist(pub, -1, 0, 0, -1)  # Înapoi și rotație la dreapta
        elif key == ',':
            sendTwist(pub, -1, 0, 0, 1)  # Înapoi și rotație la stânga
        elif key == '.':
            sendTwist(pub, 0, 0, 0, 0)  # Oprire

        rate.sleep()

if __name__ == '__main__':
    try:
        teleopKeyboard()
    except rospy.ROSInterruptException:
        pass