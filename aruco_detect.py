import roslaunch

class ArucoDetect:
    def __init__(self, camera_name, image_name, fiducial_len=0.2, dictionary=7):

        self.camera_name = camera_name
        self.image_name = image_name
        self.fiducial_len = fiducial_len
        self.dictionary = dictionary
        self.running = False


    def start(self):

        try:
            if not self.running:

                args = ['aruco_detect', 'aruco_detect.launch', 'camera:={}'.format(self.camera_name), 'image:={}'.format(self.image_name), 'fiducial_len:={}'.format(self.fiducial_len), 'dictionary:={}'.format(self.dictionary)]

                roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
                roslaunch_args = args[2:]
                
                self.parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])
                self.parent.start()
                self.running = True     

        except Exception as ex: 
                print(ex)
                self.running == False


    def stop(self):
        self.parent.shutdown()
        self.running == False


if __name__ == '__main__':
    print(1)
    import roslaunch
    import rospy
    import time


    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospy.init_node("aruco", anonymous=True)

    print(2)

    t1 = ArucoDetect(camera_name="/sim_ros_interface/camera1", image_name='image_raw', dictionary=9)
    t2 = ArucoDetect(camera_name="/sim_ros_interface/camera2", image_name='image_raw', dictionary=9)
    t1.start()
    t2.start()

    print(3)

    rospy.spin()
    