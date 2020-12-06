import rospy
from my_robot_common.import_me_if_you_can import say_it_works
if __name__ == '__main__':
    rospy.init_node('test_node')
    say_it_works()
