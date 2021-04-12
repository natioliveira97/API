import rospy
from tf2_msgs.msg import TFMessage
from pyquaternion import Quaternion
import numpy as np

class Camera:
    def __init__(self, position, rotation):
        self.position = position
        self.rotation = rotation

class MapCameras:
    def __init__(self, cameras, markers, located_markers):
        self.cameras = cameras
        self.markers = markers
        self.located_markers = {}
        for fiducial in located_markers.keys():
            realq = located_markers[fiducial]['r']
            realt = located_markers[fiducial]['t']
            realq = Quaternion(w=realq['w'],x=realq['x'],y=realq['y'],z=realq['z'])
            realt = np.array([realt['x'],realt['y'],realt['z']])
            self.located_markers[fiducial]={'r':realq, 't':realt}



        self.measures = {}
        self.located_cameras = {}
        self.unlocated_cameras = []

        for camera in self.cameras:
            self.measures[camera]={}


    def measure(self, time):
        print("iniciou")
        rospy.Subscriber("/tf", TFMessage, self.callback)
        rospy.sleep(time)
        print("acabou")

    def callback(self, data):
        for transform in data.transforms:
            if transform.child_frame_id in self.measures[transform.header.frame_id].keys():
                self.measures[transform.header.frame_id][transform.child_frame_id].append({'t':transform.transform.translation, 'r':transform.transform.rotation})
            else:
                self.measures[transform.header.frame_id][transform.child_frame_id] = [{'t':transform.transform.translation, 'r':transform.transform.rotation}]

    def compute(self):
        self.unlocated_cameras = list(self.measures)
        print(self.unlocated_cameras)

        while len(self.unlocated_cameras)>0:
            n_unknown = len(self.unlocated_cameras)
            for camera in self.unlocated_cameras:
                for fiducial in self.measures[camera].keys():
                    if fiducial in self.located_markers.keys():
                        fiducialq = self.measures[camera][fiducial][0]['r']
                        fiducialt = self.measures[camera][fiducial][0]['t']
                        fiducialt = np.array([fiducialt.x,fiducialt.y,fiducialt.z])
                        fiducialq = Quaternion(w=fiducialq.w,x=fiducialq.x,y=fiducialq.y,z=fiducialq.z)
                        
                        realq = self.located_markers[fiducial]['r']
                        realt = self.located_markers[fiducial]['t']

                        cameraq = fiducialq.inverse*realq
                        camerat = fiducialq.inverse.rotate(-fiducialt)+realt

                        self.located_cameras[camera]={'r':cameraq, 't':camerat}

                        self.unlocated_cameras.remove(camera)

                        for fiducial in self.measures[camera].keys():
                            if fiducial not in self.located_markers.keys():
                                fiducialq = self.measures[camera][fiducial][0]['r']
                                fiducialt = self.measures[camera][fiducial][0]['t']
                                fiducialt = np.array([fiducialt.x,fiducialt.y,fiducialt.z])
                                fiducialq = Quaternion(w=fiducialq.w,x=fiducialq.x,y=fiducialq.y,z=fiducialq.z)

                                realq = cameraq*fiducialq
                                realt = cameraq.rotate(fiducialt)+camerat
                                self.located_markers[fiducial]={'r':realq, 't':realq}

                        break
            
            if len(self.unlocated_cameras)==n_unknown:
                break

        print(self.located_cameras)

                                

                    



rospy.init_node('listener', anonymous=True)
mp = MapCameras(['camera1', 'camera2'], [23], {'fiducial_23':{'t':{'x':0,'y':0,'z':0},'r':{'x':0,'y':0,'z':0,'w':1}}})
mp.measure(1)

mp.compute()


