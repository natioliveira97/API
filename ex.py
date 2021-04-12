# import rospy
# from std_msgs.msg import String
# from threading import Thread

# class Teste:
#     def __init__(self, name):

#         self.name = name
#         thread = Thread(target=self.run)
#         thread.start()

#     def run(self):
#         pub = rospy.Publisher("topic_"+self.name, String, queue_size=10)
#         rospy.init_node("node_"+self.name)
#         r = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             pub.publish("hello world "+self.name)
#             print(self.name)
#             r.sleep()

#         print("Acabou")


# Teste("oi")
# print("Passou")


import roslaunch

package = 'openni_tracker'
executable = 'openni_tracker'
args='_desired_serial:=A00363904314053A _frame_id:=kinect1'
node = roslaunch.core.Node(package, executable, args=args)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print(process.is_alive())

import rospy
rospy.init_node("test_tracker", anonymous=True)
rospy.spin()
# process.stop()