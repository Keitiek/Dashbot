import rospy
from std_msgs.msg import String

print('sanity check')

rospy.init_node('your_node_name')

def callback(data):
    rospy.loginfo("Received: %s", data.data)


rospy.Subscriber("your_topic_name", String, callback)

rospy.spin()
