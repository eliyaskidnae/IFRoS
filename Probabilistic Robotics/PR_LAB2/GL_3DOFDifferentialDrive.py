from GL import GL
from DifferentialDriveSimulatedRobot import *
from DR_3DOFDifferentialDrive import *
from scipy import stats
from Pose3D import *
from Histogram import *
from scipy.ndimage import gaussian_filter
from scipy.stats import norm

class GL_3DOFDifferentialDrive(GL, DR_3DOFDifferentialDrive):
    """
    Grid Reckoning Localization for a 3 DOF Differential Drive Mobile Robot.
    """

    def __init__(self, dx_max, dy_max, range_dx, range_dy, p0, index, kSteps, robot, x0, *args):
        """
        Constructor of the :class:`GL_4DOFAUV` class. Initializes the Dead reckoning localization algorithm as well as the histogram filter algorithm.

        :param dx_max: maximum x displacement in meters
        :param dy_max: maximum y displacement in meters
        :param range_dx: range of x displacements in meters
        :param range_dy: range of y displacements in meters
        :param p0: initial probability histogram
        :param index: index struture containing plotting information
        :param kSteps: number of time steps to simulate the robot motion
        :param robot: robot object
        :param x0: initial robot pose
        :param args: additional arguments
        """

        super().__init__(p0, index, kSteps, robot, x0, *args)

        self.sigma_d = 1

        self.range_dx = range_dx
        self.range_dy = range_dy

        self.Deltax = 0  # x displacement since last x cell motion
        self.Deltay = 0  # y displacement since last y cell motion
        self.Delta_etak = Pose3D(np.zeros((3, 1)))

        self.cell_size = self.pk_1.cell_size_x  # cell size is the same for x and y

    def GetMeasurements(self):
        """
        Read the measurements from the robot. Returns a vector of range distances to the map features.
        Only those features that are within the :attr:`SimulatedRobot.SimulatedRobot.Distance_max_range` of the sensor are returned.
        The measurements arribe at a frequency defined in the :attr:`SimulatedRobot.SimulatedRobot.Distance_feature_reading_frequency` attribute.

        :return: vector of distances to the map features
        """
        # TODO: To be implemented by the student

        return self.robot.ReadRanges()

    def StateTransitionProbability_4_uk(self,uk):

        return self.Pk[uk[0, 0], uk[1, 0]]

    def StateTransitionProbability_4_xk_1_uk(self, etak_1, uk):
        """
        Computes the state transition probability histogram given the previous robot pose :math:`\eta_{k-1}` and the input :math:`u_k`:

        .. math::

            p(\eta_k | \eta_{k-1}, u_k)

        :param etak_1: previous robot pose in cells
        :param uk: input displacement in number of cells
        :return: state transition probability :math:`p_k=p(\eta_k | \eta_{k-1}, u_k)`

        """

        # TODO: To be implemented by the student

        pass

    def StateTransitionProbability(self):
        """
        Computes the complete state transition probability matrix. The matrix is a :math:`n_u \times m_u \times n^2` matrix,
        where :math:`n_u` and :math:`m_u` are the number of possible displacements in the x and y axis, respectively, and
        :math:`n` is the number of cells in the map. For each possible displacement :math:`u_k`, each previous robot pose
        :math:`{x_{k-1}}` and each current robot pose :math:`{x_k}`, the probability :math:`p(x_k|x_{k-1},u_k)` is computed.


        :return: state transition probability matrix :math:`P_k=p{x_k|x_{k-1},uk}`
        """

        # TODO: To be implemented by the student

        pass

    def uk2cell(self, uk):
        """
        Converts the number of cells the robot has displaced along its DOFs in the world N-Frame to an index that can be
        used to acces the state transition probability matrix.

        :param uk: vector containing the number of cells the robot has displaced in all the axis of the world N-Frame
        :returns: index: index that can be used to access the state transition probability matrix
        """

        # TODO: To be implemented by the student
        
        cell_size = abs(self.x_range[0])
        # You may need to consider the grid resolution (cell size) when calculating the index.
        index_x = int(uk[0][0]*cell_size)
        index_y = int(uk[1][0]*cell_size)
        index = [[index_x] , [index_y]]
        
        return index 
        pass

    def MeasurementProbability(self, zk):
        
        """
        Computes the measurement probability histogram given the robot pose :math:`\eta_k` and the measurement :math:`z_k`.
        In this case the measurement is the vector of the distances to the landmarks in the map.

        :param zk: :math:`z_k=[r_0~r_1~..r_k]` where :math:`r_i` is the distance to the i-th landmark in the map.
        :returns: Measurement probability histogram :math:`p_z=p(z_k | \eta_k)`
        """

        # TODO: To be implemented by the student
        # measurement_histogram = np.zeros((self.num_bins_x,self.num_bins_y))
        mh = Histogram2D(self.num_bins_x, self.num_bins_y, self.x_range, self.y_range)
        xsk = self.robot.xsk
        # eta_x = xsk[0][0]
        # eta_y = xsk[1][0]
        for iz,z in enumerate(zk):
            # print("First Landmark")
            m1,m2= self.robot.M[iz]

            for x in mh.x_range:
                for y in mh.y_range:                  
                    m1,m2= self.robot.M[iz]
                    # print("x,y" ,x,y)
                    d = math.hypot( x - m1[0], y - m2[0])
                    pdf = stats.norm.pdf(d,z,scale=self.sigma_d)
                    mh.element[x,y] = mh.element[x,y]+ pdf
                                                  
        return mh
      

    def GetInput(self,usk):
        """
        Provides an implementation for the virtual method :meth:`GL.GetInput`.
        Gets the number of cells the robot has displaced in the x and y directions in the world N-Frame. To do it, it
        calls several times the parent method :meth:`super().GetInput`, corresponding to the Dead Reckoning Localization
        of the robot, until it has displaced at least one cell in any direction.
        Note that an iteration of the robot simulation :meth:`SimulatedRobot.fs` is normally done in the :meth:`GL.LocalizationLoop`
        method of the :class:`GL.Localization` class, but in this case it is done here to simulate the robot motion
        between the consecutive calls to :meth:`super().GetInput`.

        :param usk: control input of the robot simulation
        :return: uk: vector containing the number of cells the robot has displaced in the x and y directions in the world N-Frame
        """
        # TODO: To be implemented by the student
        # xsk_1 = self.robot.xsk_1
        # xsk = self.robot.fs(xsk_1, usk)  # Simulate the robot motion
        
        # super().GetInput() 
        x_shift = 0
        y_shift = 0
        dx =0
        dy =0
        xsk_1 = self.robot.xsk
        cell_size_x = self.p0.cell_size_x
        cell_size_y = self.p0.cell_size_y
        while(-1<x_shift<1 and -1<y_shift<1):
            
            xsk = self.robot.fs(xsk_1, usk)
            dx += xsk[0][0] - xsk_1[0][0]       # change in x displacement
            dy += xsk[1][0] - xsk_1[1][0]       # change in y displacement
            x_shift = dx // cell_size_x        # divide by the cell size
            y_shift = dy // cell_size_y        # divide by the cell size
            xsk_1 = xsk
            # print(xsk_1 , xsk_1 )
            print("\n")
            print(dx ,dy , x_shift ,x_shift , cell_size_x)
            
        no_cells_disp = [x_shift , y_shift]
            
     
        return no_cells_disp
        # return [[0],[1]]
        # pass



