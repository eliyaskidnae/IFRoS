import dis
import math
from SimulatedRobot import *
from IndexStruct import *
from Pose3D import *
import scipy
from roboticstoolbox.mobile.Animations import *
import numpy as np
from math import sqrt
    


class DifferentialDriveSimulatedRobot(SimulatedRobot):
    """
    This class implements a simulated differential drive robot. It inherits from the :class:`SimulatedRobot` class and
    overrides some of its methods to define the differential drive robot motion model.
    """
    def __init__(self, xs0, map=[],*args):
        """
        :param xs0: initial simulated robot state 
           :math:`\\mathbf{x_{s_0}}=[^Nx{_{s_0}}~^Ny{_{s_0}}~^N\psi{_{s_0}}~]^T` used to initialize the  motion model
        :param map: feature map of the environment :math:`M=[^Nx_{F_1},...,^Nx_{F_{nf}}]`

        Initializes the simulated differential drive robot. Overrides some of the object attributes of the parent class :class:`SimulatedRobot` to define the differential drive robot motion model:

        * **Qsk** : Object attribute containing Covariance of the simulation motion model noise.

        .. math::
            Q_k=\\begin{bmatrix}\\sigma_{\\dot u}^2 & 0 & 0\\\\
            0 & \\sigma_{\\dot v}^2 & 0 \\\\
            0 & 0 & \\sigma_{\\dot r}^2 \\\\
            \\end{bmatrix}
            :label: eq:Qsk

        * **usk** : Object attribute containing the simulated input to the motion model containing the forward velocity :math:`u_k` and the angular velocity :math:`r_k`

        .. math::
            \\bf{u_k}=\\begin{bmatrix}u_k & r_k\\end{bmatrix}^T
            :label: eq:usk

        * **xsk** : Object attribute containing the current simulated robot state

        .. math::
            x_k=\\begin{bmatrix}{^N}x_k & {^N}y_k & {^N}\\theta_k & {^B}u_k & {^B}v_k & {^B}r_k\\end{bmatrix}^T
            :label: eq:xsk

        where :math:`{^N}x_k`, :math:`{^N}y_k` and :math:`{^N}\\theta_k` are the robot position and orientation in the world N-Frame, and :math:`{^B}u_k`, :math:`{^B}v_k` and :math:`{^B}r_k` are the robot linear and angular velocities in the robot B-Frame.

        * **zsk** : Object attribute containing :math:`z_{s_k}=[n_L~n_R]^T` observation vector containing number of pulses read from the left and right wheel encoders.
        * **Rsk** : Object attribute containing :math:`R_{s_k}=diag(\\sigma_L^2,\\sigma_R^2)` covariance matrix of the noise of the read pulses`.
        * **wheelBase** : Object attribute containing the distance between the wheels of the robot (:math:`w=0.5` m)
        * **wheelRadius** : Object attribute containing the radius of the wheels of the robot (:math:`R=0.1` m)
        * **pulses_x_wheelTurn** : Object attribute containing the number of pulses per wheel turn (:math:`pulseXwheelTurn=1024` pulses)
        * **Polar2D_max_range** : Object attribute containing the maximum Polar2D range (:math:`Polar2D_max_range=50` m) at which the robot can detect features.
        * **Polar2D\_feature\_reading\_frequency** : Object attribute containing the frequency of Polar2D feature readings (50 tics -sample times-)
        * **Rfp** : Object attribute containing the covariance of the simulated Polar2D feature noise (:math:`R_{fp}=diag(\\sigma_{\\rho}^2,\\sigma_{\\phi}^2)`)

        Check the parent class :class:`prpy.SimulatedRobot` to know the rest of the object attributes.
        """
        super().__init__(xs0, map,*args) # call the parent class constructor

        # Initialize the motion model noise
        self.Qsk = np.diag(np.array([0.1 ** 2, 0.01 ** 2, np.deg2rad(1) ** 2]))  # simulated acceleration noise
        self.usk = np.zeros((3, 1))  # simulated input to the motion model

        # Inititalize the robot parameters
        self.wheelBase = 0.5  # distance between the wheels
        self.wheelRadius = 0.1  # radius of the wheels
        self.pulse_x_wheelTurns = 1024  # number of pulses per wheel turn

        # Initialize the sensor simulation
        self.encoder_reading_frequency = 1  # frequency of encoder readings
        self.Re= np.diag(np.array([22 ** 2, 22 ** 2]))  # covariance of simulated wheel encoder noise

        self.Polar2D_feature_reading_frequency = 50  # frequency of Polar2D feature readings
        self.Polar2D_max_range = 50  # maximum Polar2D range, used to simulate the field of view
        self.Rfp = np.diag(np.array([1 ** 2, np.deg2rad(5) ** 2]))  # covariance of simulated Polar2D feature noise

        self.xy_feature_reading_frequency = 50  # frequency of XY feature readings
        self.xy_max_range = 50  # maximum XY range, used to simulate the field of view

        # self.yaw_reading_frequency = 10  # frequency of Yasw readings
        self.yaw_reading_frequency = 1  # frequency of Yasw readings
        self.v_yaw_std = np.deg2rad(5)  # std deviation of simulated heading noise

    def fs(self, xsk_1, usk):  # input velocity motion model with velocity noise
       
        # TODO: to be completed by the student
          
        # previous pose and velocity
        eta_s_k_1 = Pose3D(np.array(np.array(xsk_1[:3])))
        nu_s_k_1 = Pose3D(np.array(np.array(xsk_1[3:6])))
        # desired velocity
        
        k=np.diag((1,1,1))   # 3x3 
        # xsk = np.zeros((6, 1))
        wsk =np.array(
              [
            [np.random.normal(0, 0.01)],
            [0],
            [np.random.normal(0, 0.01)]
        ]
        )
        
        usk = Pose3D(np.array([[usk[0, 0]], [0] ,  [usk[1, 0]]]))#  3x1
        self.usk = usk
        # current pose and velocity
        eta_s_k = eta_s_k_1.oplus(Pose3D(np.array(nu_s_k_1 * self.dt + 0.5 * wsk * self.dt*self.dt))) # 3x1
        nu_s_k  = nu_s_k_1 +  k@(usk - nu_s_k_1) + wsk * self.dt # 3x1

        # current robot state
        self.xsk = np.concatenate((eta_s_k, nu_s_k))
        
        # print("xsk",self.xsk)
               
        if self.k % self.visualizationInterval == 0:
                self.PlotRobot()
                self.xTraj.append(self.xsk[0, 0])
                self.yTraj.append(self.xsk[1, 0])
                self.trajectory.pop(0).remove()
                self.trajectory = plt.plot(self.xTraj, self.yTraj, marker='.', color='red', markersize=1)


        self.k += 1
        
     
        return self.xsk


    def ReadEncoders(self):
        """ Simulates the robot measurements of the left and right wheel encoders.

        **To be completed by the student**.

        :return zsk,Rsk: :math:`zk=[n_L~n_R]^T` observation vector containing number of pulses read from the left and right wheel encoders. :math:`R_{s_k}=diag(\\sigma_L^2,\\sigma_R^2)` covariance matrix of the read pulses.
        """

        # TODO: to be completed by the student
        dt = self.dt
        wheelBase = self.wheelBase
        wheelRadius = self.wheelRadius
        pulse_x_wheelTurns = self.pulse_x_wheelTurns
        
        linear_velocity = self.xsk[3][0] # get the linear velocity from desired velocity
        angular_velocity = self.xsk[5][0]# angular velocity in rad/s

        #if we assume the rotational angle is at the mid of the two wheels 
        
        dl =(linear_velocity - angular_velocity*(wheelBase/2))*dt # Left Wheel Distance 
        dr =(linear_velocity + angular_velocity*(wheelBase/2))*dt # Right Wheel Distance 
        
        # print(linear_velocity ,dl ,dr , dt)
        left_wheel_rot_pulse = (dl*pulse_x_wheelTurns)/(2*pi*wheelRadius)
        right_wheel_rot_pulse = (dr*pulse_x_wheelTurns)/(2*pi*wheelRadius)
        
        zsk  = np.array([[left_wheel_rot_pulse] , [right_wheel_rot_pulse]])
        
        noise =np.array(
              [
            [np.random.normal(0,1)],
            [np.random.normal(0,1)]
        ]
        )
        zsk = zsk 
        rsk = self.Re
        
        return zsk,rsk
    
    def ReadCompass(self):
        """ Simulates the compass reading of the robot.

        :return: yaw and the covariance of its noise *R_yaw*
        """

        # TODO: to be completed by the student
        yaw = self.xsk_1[3]
        wc = np.random.normal([0,yaw])
        R_yaw = np.diag([self.v_yaw_std**2])
        
        return yaw , R_yaw

    def ReadRanges(self):
        map = self.M
        xs1 , ys1 = self.xsk[0:2]
        distance_range = np.array([])  
        for i in range(len(map)):
            m1 , m2 = map[i]
            # print(m1[0],m2[0])
            dx = m1[0] - xs1
            dy = m2[0] - ys1
            distance = math.hypot(dx,dy) # calculate equlidian  distance   
            distance_range = np.append(distance_range,distance)
         
        noise = [
            [np.random.normal(0, 0.01)],
            [0],
            [np.random.normal(0, 0.01)]
        ]
        
        
        return distance_range 
        
    def PlotRobot(self):
        """ Updates the plot of the robot at the current pose """

        self.vehicleIcon.update([self.xsk[0], self.xsk[1], self.xsk[2]])
        plt.pause(0.0000001)
        return

#  Doppler Velocity Log (DVL) 