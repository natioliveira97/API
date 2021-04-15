import roslaunch

class SimCamera:
    def __init__(self, camera_name):

        self.camera_name = camera_name
        self.running = False


    def start(self):

        try:
            if not self.running:
                package = 'image_transport'
                executable = 'republish'
                args='raw in:=/sim_ros_interface/'+self.camera_name+'/image_raw'+' compressed out:=/sim_ros_interface/'+self.camera_name+'/image_raw'
                node = roslaunch.core.Node(package, executable, args=args)

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                process = launch.launch(node)
                self.running = True     

        except: 
                self.running == False


    def stop(self):
        self.parent.shutdown()
        self.running == False


if __name__ == '__main__':
    import roslaunch
    import rospy
    import time

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospy.init_node("teste", anonymous=True)
    cam1 = SimCamera(camera_name='camera1')
    cam1.start()
    cam2 = SimCamera(camera_name='camera2')
    cam2.start()
    cam3 = SimCamera(camera_name='camera3')
    cam3.start()



    rospy.spin()