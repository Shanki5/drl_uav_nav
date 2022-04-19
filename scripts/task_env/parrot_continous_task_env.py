from openai_ros.task_envs import parrotdrone_goto


class ParrotDroneGotoContinuous(parrotdrone_goto.ParrotDroneGotoEnv):
    def __init__(self):
        """
        Make parrotdrone learn how to navigate to get to a point
        """
        ros_ws_abspath = rospy.get_param("/drone/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="drone_construct",
                    launch_file_name="start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/parrotdrone/config",
                               yaml_file_name="parrotdrone_goto.yaml")

        # Only variable needed to be set here
        #number_actions = rospy.get_param('/drone/n_actions')
        self.action_space = spaces.Box(low = [-1, -1, -1, -1], high = [1, 1, 1, 1])

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param(
            '/drone/linear_forward_speed')
        self.angular_turn_speed = rospy.get_param('/drone/angular_turn_speed')
        self.angular_speed = rospy.get_param('/drone/angular_speed')

        self.init_linear_speed_vector = Vector3()
        self.init_linear_speed_vector.x = rospy.get_param(
            '/drone/init_linear_speed_vector/x')
        self.init_linear_speed_vector.y = rospy.get_param(
            '/drone/init_linear_speed_vector/y')
        self.init_linear_speed_vector.z = rospy.get_param(
            '/drone/init_linear_speed_vector/z')

        self.init_angular_turn_speed = rospy.get_param(
            '/drone/init_angular_turn_speed')

        self.min_sonar_value = rospy.get_param('/drone/min_sonar_value')
        self.max_sonar_value = rospy.get_param('/drone/max_sonar_value')

        # Get WorkSpace Cube Dimensions
        self.work_space_x_max = rospy.get_param("/drone/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/drone/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/drone/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/drone/work_space/y_min")
        self.work_space_z_max = rospy.get_param("/drone/work_space/z_max")
        self.work_space_z_min = rospy.get_param("/drone/work_space/z_min")

        # Maximum RPY values
        self.max_roll = rospy.get_param("/drone/max_roll")
        self.max_pitch = rospy.get_param("/drone/max_pitch")
        self.max_yaw = rospy.get_param("/drone/max_yaw")

        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/drone/desired_pose/x")
        self.desired_point.y = rospy.get_param("/drone/desired_pose/y")
        self.desired_point.z = rospy.get_param("/drone/desired_pose/z")

        self.desired_point_epsilon = rospy.get_param(
            "/drone/desired_point_epsilon")

        # We place the Maximum and minimum values of the X,Y,Z,R,P,Yof the pose

        high = numpy.array([self.work_space_x_max,
                            self.work_space_y_max,
                            self.work_space_z_max,
                            self.max_roll,
                            self.max_pitch,
                            self.max_yaw,
                            self.max_sonar_value])

        low = numpy.array([self.work_space_x_min,
                           self.work_space_y_min,
                           self.work_space_z_min,
                           -1*self.max_roll,
                           -1*self.max_pitch,
                           -numpy.inf,
                           self.min_sonar_value])

        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))

        # Rewards
        self.closer_to_point_reward = rospy.get_param(
            "/drone/closer_to_point_reward")
        self.not_ending_point_reward = rospy.get_param(
            "/drone/not_ending_point_reward")
        self.end_episode_points = rospy.get_param("/drone/end_episode_points")

        self.cumulated_steps = 0.0

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(ParrotDroneGotoEnv, self).__init__(ros_ws_abspath)

    def set_actions(self, action):
        linear_speed_vector = [action[0], action[1], action[2]]
        angular_speed = action[3]
        self.move_base(linear_speed_vector,
                       angular_speed,
                       epsilon=0.05,
                       update_rate=10)