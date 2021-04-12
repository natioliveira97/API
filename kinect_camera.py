import roslaunch

class KinectCamera:
    def __init__(self, camera_name, device_id, rgb_camera_info_url = None, depth_camera_info_url = None):

        self.camera_name = camera_name
        self.device_id = device_id

        if rgb_camera_info_url != None:
            self.calibrated = True

        else:
            self.calibrated = False

        self.rgb_camera_info_url = rgb_camera_info_url
        self.depth_camera_info_url = depth_camera_info_url

        self.running = False


    def start(self):

        try:
            if not self.running:
        
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                rospy.init_node(self.camera_name, anonymous=True)

                args = ['openni_launch', 'openni.launch', 'camera:={}'.format(self.camera_name), 'device_id:={}'.format(self.device_id)]
                if self.rgb_camera_info_url is not None:
                    args = args + ['rgb_camera_info_url:={}'.format(self.rgb_camera_info_url)] 
                if self.depth_camera_info_url is not None:
                    args = args + ['depth_camera_info_url:={}'.format(self.depth_camera_info_url)] 

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
    try:
        import roslaunch
        import rospy
        import time

        print(2)

        kinect = KinectCamera(camera_name="kinect1", device_id='A00363904314053A')
        kinect.start()

        print(3)
        rospy.spin()
    except Exception as ex:
        print(ex)
        


            