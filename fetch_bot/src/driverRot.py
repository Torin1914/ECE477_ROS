import rospy

def callback(data):
    rospy.loginfo("I heard %s",data.data)
     
def listener()
    rospy.init_node('node_name')
   9     rospy.Subscriber("chatter", String, callback)
  10     # spin() simply keeps python from exiting until this node is stopped
  11     rospy.spin()
