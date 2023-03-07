import numpy as np
from scipy import io
from quaternion import Quaternion
import math
from matplotlib import pyplot as plt
import scipy

#data files are numbered on the server.
#for exmaple imuRaw1.mat, imuRaw2.mat and so on.
#write a function that takes in an input number (1 through 6)
#reads in the corresponding imu Data, and estimates
#roll pitch and yaw using an unscented kalman filter

def estimate_rot(data_num=1):
    #load data
    imu = io.loadmat('imu/imuRaw'+str(data_num)+'.mat')
    # vicon = io.loadmat('vicon/viconRot'+str(data_num)+'.mat')
    accel = imu['vals'][0:3,:]
    gyro = imu['vals'][3:6,:]
    T = np.shape(imu['ts'])[1]

    # your code goes here
    num_to_debug = 500
    imu_t = imu['ts']
    
    bias = np.array([510.808, 500.994, 505.901])

    sens_final = 32.64

    sensitivities = np.array([sens_final, sens_final, sens_final])

    new_acc = np.zeros(accel.shape)
    new_acc[0] = -1*(accel[0,:]-bias[0])*3300/1023/sensitivities[0]
    new_acc[1] = -1*(accel[1,:]-bias[1])*3300/1023/sensitivities[1]
    new_acc[2] = (accel[2,:]-bias[2])*3300/(1023*sensitivities[2])
    

    bias_gyro = np.mean(gyro[:,:num_to_debug], 1)


    new_gyro = np.zeros(gyro.shape)
    sens_gyro = np.array([184.84693891, 222.32377482, 154.89639974])
    new_gyro[0] = (gyro[1,:]-bias_gyro[1])*3300/1023/sens_gyro[1]
    new_gyro[1] = (gyro[2,:]-bias_gyro[2])*3300/1023/sens_gyro[2]
    new_gyro[2] = (gyro[0,:]-bias_gyro[0])*3300/1023/sens_gyro[0]



    q_init = Quaternion()
    w_init = 0.4 * np.ones(3)
    x_init = np.hstack((q_init.q, w_init))
    n = 6
    cov_init = 0.3 * np.diag(np.ones(6))
    Q = np.diag([0.01,0.01,0.01, 0.3, 0.3, 0.3])
    R = np.diag([8, 8, 8, 4, 4, 4]) 
    roll = []
    pitch = []
    yaw = []
    print(T)

    eulers_init = Quaternion(x_init[0], x_init[1:4])
    eulers_init_angles = eulers_init.euler_angles()
    roll.append(eulers_init_angles[0])
    pitch.append(eulers_init_angles[1])
    yaw.append(eulers_init_angles[2])
    innovation = []
    xhat = [x_init]
    phat = [cov_init]
    for i in range(1,T):
        
        delta_t = imu['ts'][0][i] - imu['ts'][0][i-1]
        # print(delta_t)
        S = scipy.linalg.sqrtm(n*(cov_init + Q))
        W = np.hstack((S, -S))
        Xi = np.zeros((W.shape[0]+1, W.shape[1])) #sigma points
        q_delta = Quaternion()
        q_delta.from_axis_angle(x_init[4:]*delta_t)
        for j in range(2*n):
            
            q_W = Quaternion()
            q_W.from_axis_angle(W[0:3, j])
            
            q_temp = Quaternion(x_init[0], x_init[1:4].reshape((3)))*q_W
            Xi[0:4,j] = (q_temp*q_delta).q
            Xi[4:,j] = w_init.reshape((3)) + W[3:, j].reshape((3))
        prev_q = Quaternion(Xi[0,0], Xi[1:4, 0].reshape(3,))
        num_iter = 100
        err_vectors = np.zeros((3,12))
        for j in range(num_iter):
            for k in range(Xi.shape[1]):
                err_temp = Quaternion(Xi[0, k], Xi[1:4, k])*prev_q.inv()
                if err_temp.scalar()> 1.0 and abs(err_temp.scalar() - 1.0) < 0.0001:
                    err_temp = Quaternion(1.0, err_temp.vec())
                if err_temp.scalar() < -1.0 and abs(err_temp.scalar() + 1.0) < 0.0001:
                    err_temp = Quaternion(-1.0, err_temp.vec())
                err_vectors[:, k] = err_temp.axis_angle()
            
            err_mean = np.mean(err_vectors, axis=1)
            err_mean_quat = Quaternion()
            err_mean_quat.from_axis_angle(err_mean)

            prev_q = err_mean_quat*prev_q
            # print("q after each iter {$j}: ", new_q.q)
            if np.all(abs(err_mean) < 0.01):    
                # print("stopped at iter : ", j)
                break
        
        W_prime = np.zeros((W.shape))  
        W_prime[0:3, :] = err_vectors[:, :]
        w_mean = np.mean(Xi[4:,:], axis=1)
        W_prime[3:, :] = Xi[4:, :] - w_mean.reshape((-1,1))
        x_init = np.hstack([prev_q.q, np.mean(Xi[4:7, :], axis=1)])
        w_init = x_init[4:]
        cov_init = (W_prime @ W_prime.T)/12


        #measurement model
        Zi = get_measurement_vectors(Xi)
        Z_mean = np.mean(Zi, axis=1).reshape((-1,1))
        
        Pzz = (Zi - Z_mean)@((Zi - Z_mean).T)/12
        Pxz = (W_prime)@((Zi - Z_mean).T)/12
        Pvv = Pzz + R

        kalman_gain = Pxz@(np.linalg.inv(Pvv))
        new_acc_meas = np.array([-1*(accel[0,i-1]-bias[0])*3300/1023/sensitivities[0], -1*(accel[1,i-1]-bias[1])*3300/1023/sensitivities[1], (accel[2,i-1]-bias[2])*3300/(1023*sensitivities[2])])
        new_gyro_meas = np.array([(gyro[1,i-1]-bias_gyro[1])*3300/1023/sens_gyro[1], (gyro[2,i-1]-bias_gyro[2])*3300/1023/sens_gyro[2], (gyro[0,i-1]-bias_gyro[0])*3300/1023/sens_gyro[0]])
        
        accel_gyro_measures = (np.hstack((new_acc_meas, new_gyro_meas))).reshape((-1,1))
        updated = kalman_gain @ (accel_gyro_measures - Z_mean)
        innovation.append(updated[-3])


        q_updated = Quaternion()
        q_updated.from_axis_angle(updated[:3].reshape((3)))
        x_init[:4] = (q_updated*Quaternion(x_init[0], x_init[1:4].reshape((3)))).q
        x_init[4:] = x_init[4:].reshape((-1)) + updated[3:].reshape((-1))
        
        cov_init = cov_init - kalman_gain @ Pvv @ kalman_gain.T
        w_init = x_init[4:].reshape((3))
        xhat.append(x_init)
        phat.append(cov_init)
        euler_angles = Quaternion(x_init[0], x_init[1:4].reshape((3))).euler_angles()
        roll.append(euler_angles[0])
        pitch.append(euler_angles[1])
        yaw.append(euler_angles[2])
    vicon = io.loadmat('vicon/viconRot'+str(data_num)+'.mat')
    vicon_R = vicon['rots']
    vicon_alpha = np.arctan2(vicon_R[1,0,:], vicon_R[0,0,:]) #yaw
    vicon_beta = np.arctan2(-1*vicon_R[2,0,:], np.sqrt(np.square(vicon_R[2,1,:]) + np.square(vicon_R[2,2,:]))) #pitch
    vicon_gamma = np.arctan2(vicon_R[2,1,:], vicon_R[2,2,:]) #roll

    n = 5561
    xhat = np.array(xhat)
    phat = np.array(phat)

    vicon_euler, vicon_quat, vicon_axis_angle = convert_to_euler(vicon_R)

    x_axis_angle = convert_x_axis_angle(xhat)
    print(x_axis_angle)

    return roll, pitch, yaw


def convert_x_axis_angle(xhat):
    x = []
    for i in range(0, len(xhat)):
        q = Quaternion(xhat[i, 0], xhat[i, 1:4])
        out = q.axis_angle()
        x.append(out)
    return np.array(x)
        
def convert_to_euler(vicon_rot):
    vicon_euler = []
    vicon_quat = []
    vicon_axis_angle = []
    q = Quaternion()
    for i in range(vicon_rot.shape[2]):
        rot = vicon_rot[:, :, i]
        q.from_rotm(rot)
        euler = q.euler_angles()
        # euler = Rotation.from_matrix(rot).as_euler("XYZ")
        vicon_quat.append(q.q)
        vicon_euler.append(euler)
        vicon_axis_angle.append(q.axis_angle())
    return np.array(vicon_euler).T, np.array(vicon_quat).T, np.array(vicon_axis_angle).T

def get_measurement_vectors(Yi):
    g = Quaternion(0, [0,0,9.81])
    Zi = np.zeros((Yi.shape[0]-1, Yi.shape[1]))
    for k in range(Yi.shape[1]):
        qk = Quaternion(Yi[0, k], Yi[1:4, k])
        Zi[:3, k] = (qk.inv()*g*qk).vec()
    Zi[3:, :] = Yi[4:, :]
    return Zi

estimate_rot()