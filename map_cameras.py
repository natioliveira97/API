import rospy
from tf2_msgs.msg import TFMessage
from pyquaternion import Quaternion
import numpy as np
import numpy.matlib as npm
import json
import logging

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

    def _averageQuaternions(self, Q):
        # Number of quaternions to average
        M = Q.shape[0]
        A = npm.zeros(shape=(4,4))

        for i in range(0,M):
            q = Q[i,:]
            # multiply q with its transposed version q' and add A
            A = np.outer(q,q) + A

        # scale
        A = (1.0/M)*A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:,0].A1)


    def measure(self, time):
        sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        rospy.sleep(time)
        sub.unregister()

        for camera in self.measures.keys():
            for fiducial in self.measures[camera].keys():
                fiducialq = np.array([self.measures[camera][fiducial][0]['r'].w,
                                      self.measures[camera][fiducial][0]['r'].x,
                                      self.measures[camera][fiducial][0]['r'].y,
                                      self.measures[camera][fiducial][0]['r'].z])


                fiducialt = np.array([self.measures[camera][fiducial][0]['t'].x,
                                      self.measures[camera][fiducial][0]['t'].y,
                                      self.measures[camera][fiducial][0]['t'].z])



                for i in range(1,len(self.measures[camera][fiducial])):
                    fiducialq = np.vstack((fiducialq, np.array([self.measures[camera][fiducial][i]['r'].w,
                                                                self.measures[camera][fiducial][i]['r'].x,
                                                                self.measures[camera][fiducial][i]['r'].y,
                                                                self.measures[camera][fiducial][i]['r'].z])))

                    fiducialt = np.vstack((fiducialt, np.array([self.measures[camera][fiducial][i]['t'].x,
                                                                self.measures[camera][fiducial][i]['t'].y,
                                                                self.measures[camera][fiducial][i]['t'].z])))


                fiducialq = fiducialq[:,[3,0,1,2]]
                fiducialq = self._averageQuaternions(fiducialq)
                fiducialt = fiducialt.sum(axis=0)/len(self.measures[camera][fiducial])

                w,x,y,z = fiducialq
                fiducialq = {'x':x, 'y':y, 'z':z, 'w':w}

                x,y,z = fiducialt
                fiducialt = {'x':x, 'y':y, 'z':z}

                self.measures[camera][fiducial]={'r':fiducialq, 't':fiducialt}



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
                        fiducialq = self.measures[camera][fiducial]['r']
                        fiducialt = self.measures[camera][fiducial]['t']
                        fiducialt = np.array([fiducialt['x'],fiducialt['y'],fiducialt['z']])
                        fiducialq = Quaternion(w=fiducialq['w'],x=fiducialq['x'],y=fiducialq['y'],z=fiducialq['z'])
                        
                        realq = self.located_markers[fiducial]['r']
                        realt = self.located_markers[fiducial]['t']

                        cameraq = fiducialq.inverse*realq
                        camerat = fiducialq.inverse.rotate(-fiducialt)+realt

                        self.located_cameras[camera]={'r':cameraq, 't':camerat}

                        self.unlocated_cameras.remove(camera)

                        for aux_fid in self.measures[camera].keys():
                            if aux_fid not in self.located_markers.keys():
                                aux_fidq = self.measures[camera][aux_fid]['r']
                                aux_fidt = self.measures[camera][aux_fid]['t']
                                aux_fidt = np.array([aux_fidt['x'],aux_fidt['y'],aux_fidt['z']])
                                aux_fidq = Quaternion(w=aux_fidq['w'],x=aux_fidq['x'],y=aux_fidq['y'],z=aux_fidq['z'])

                                realq = cameraq*aux_fidq
                                realt = cameraq.rotate(aux_fidt)+camerat
                                self.located_markers[aux_fid]={'r':realq, 't':realt}
                        
                        break

            if len(self.unlocated_cameras)==n_unknown:
                break

    def save(self,filename, logfilename = "map_cameras.log"):
        f = open(filename, "w")
        json.dumps(self.located_cameras, f)
        f.close()

        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO) # Sets log level to INFO.

        fh = logging.FileHandler(logfilename)

        self.logger.addHandler(fh)
        self.logger.addHandler(logging.StreamHandler())

        if len(self.unlocated_cameras)>0:
            self.logger.info("As câmeras ", self.unlocated_cameras, " não foram localizadas.")

        else:
            self.logger.info("Todas as câmeras foram localizadas!")



        

                                
rospy.init_node('listener', anonymous=True)
mp = MapCameras(['camera1', 'camera2', 'camera3','camera4','camera5','camera6','camera7','camera8','camera9','camera10'], [], {'fiducial_4':{'t':{'x':0,'y':1.3,'z':0},'r':{'x':0,'y':0,'z':0,'w':1}}})
mp.measure(1)

mp.compute()

mp.save("cameras.json")

#print(mp.located_cameras)
#print(mp.located_markers)

