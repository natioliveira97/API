import roslaunch

class KinectTracker:
    def __init__(self, camera_name, device_id):

        self.camera_name = camera_name
        self.device_id = device_id
        self.running = False


    def start(self):

        try:
            if not self.running:
                print('1')
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                rospy.init_node(self.camera_name+"_tracker", anonymous=True)
                package = 'openni_tracker'
                executable = 'openni_tracker'
                args='_desired_serial:='+self.device_id+' _camera_frame_id:='+self.camera_name
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

    kinect = KinectTracker(camera_name="kinect1", device_id='A00363904314053A')
    kinect.start()

    t = time.time()


    rospy.spin()
    