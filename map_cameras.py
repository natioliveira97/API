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
        rospy.Subscriber("/tf", TFMessage, self.callback)
        rospy.sleep(time)

    def callback(self, data):
        for transform in data.transforms:
            if transform.child_frame_id in self.measures[transform.header.frame_id].keys():
                self.measures[transform.header.frame_id][transform.child_frame_id].append({'t':transform.transform.translation, 'r':transform.transform.rotation})
            else:
                self.measures[transform.header.frame_id][transform.child_frame_id] = [{'t':transform.transform.translation, 'r':transform.transform.rotation}]

    def compute(self):
        self.unlocated_cameras = list(self.measures)

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

                        for aux_fid in self.measures[camera].keys():
                            if aux_fid not in self.located_markers.keys():
                                aux_fidq = self.measures[camera][aux_fid][0]['r']
                                aux_fidt = self.measures[camera][aux_fid][0]['t']
                                aux_fidt = np.array([aux_fidt.x,aux_fidt.y,aux_fidt.z])
                                aux_fidq = Quaternion(w=aux_fidq.w,x=aux_fidq.x,y=aux_fidq.y,z=aux_fidq.z)

                                realq = cameraq*aux_fidq
                                realt = cameraq.rotate(aux_fidt)+camerat
                                self.located_markers[aux_fid]={'r':realq, 't':realt}

            if len(self.unlocated_cameras)==n_unknown:
                break


                                

                    



rospy.init_node('listener', anonymous=True)
mp = MapCameras(['camera1', 'camera2', 'camera3'], [23], {'fiducial_4':{'t':{'x':0,'y':1.3,'z':0},'r':{'x':0,'y':0,'z':0,'w':1}}})
mp.measure(1)

mp.compute()


