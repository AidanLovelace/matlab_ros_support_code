classdef RobotHandle < handle
    properties
        % Connection properties
        MasterHostIP
        NodeIP
        IsConnected = false

        options

        objectIdentifier

        % TF properties
        TFListeningTime = 10

        % Subscribers
        joint_state_sub;

        % Services
        res_client;
        get_models_client;
        get_models_state_client;

        % Actions
        point;
        TimeFromStart;
        trajPts;
        trajPtsVar;

        gripControllerClient;
        jointControllerClient;

        % Robot
        UR5eROBOT;
        initialRobotJConfig;

        % IK
        ik;
        ik_weights;
        gik;
        gik_weights;
        jointCon;

        % TF
        tftree;
        tf_listening_time;

        % Images
        rgb_sub;
        caminfo_sub;
        pt_cloud_sub;
    end

    methods
        function this = RobotHandle(options, masterHostIP, nodeIP)
            % Constructor with default values
            if nargin < 2
                this.MasterHostIP = "100.113.159.11";
            else
                this.MasterHostIP = masterHostIP;
            end

            if nargin < 3
                this.NodeIP = "100.127.95.110";
            else
                this.NodeIP = nodeIP;
            end

            this.options = options;
        end

        function connect(this)
            % Establish connection to ROS Master Node
            cprintf('blue','Establishing Connection to ROS Master Node...\n');

            if ros.internal.Global.isNodeActive() == 1
                % ROS Global Node is running
                % Get the node handle
                node = ros.internal.Global.getNodeHandle();

                if contains(node.MasterURI, this.MasterHostIP)
                    cprintf('text', ' - ROS Global Node already running on: %s\n', node.MasterURI);
                else
                    cprintf('text', ' - ROS Global Node was already running on: %s\n', node.MasterURI);
                    cprintf('text', ' - Restarting global node to run on %s...\n', this.MasterHostIP);
                    rosshutdown;
                    rosinit(this.MasterHostIP, 11311, "NodeHost", this.NodeIP);
                end
            else
                cprintf('text', ' - Starting ROS Global Node on %s...\n', this.MasterHostIP);
                rosinit(this.MasterHostIP, 11311, "NodeHost", this.NodeIP);
            end
            cprintf('text', ' - ROS Global Node ready!\n');

            % Initialize UR5e robot components
            this.initializeRobotComponents();
            this.IsConnected = true;
        end

        function initializeRobotComponents(this)
            this.joint_state_sub         = rossubscriber("/joint_states");

            % Services
            this.res_client              = rossvcclient('/gazebo/reset_world', 'std_srvs/Empty', 'DataFormat', 'struct');

            this.get_models_client       = rossvcclient('/gazebo/get_world_properties', 'DataFormat','struct');
            this.get_models_state_client = rossvcclient('/gazebo/get_model_state','DataFormat','struct');

            % Actions
            this.TimeFromStart           = rosduration(0.5,'DataFormat','struct');

            this.point                   = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat','struct');
            this.trajPts                 = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat', 'struct');
            this.trajPtsVar              = rosmessage('trajectory_msgs/JointTrajectoryPoint');

            this.gripControllerClient      = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                        'control_msgs/FollowJointTrajectory',...
                                                        'DataFormat','struct');
            this.gripControllerClient.ActivationFcn = [];
            this.gripControllerClient.FeedbackFcn = [];
            this.gripControllerClient.ResultFcn = [];

            this.jointControllerClient    = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                                       'control_msgs/FollowJointTrajectory', ...
                                                       'DataFormat', 'struct');
            this.jointControllerClient.ActivationFcn = [];
            this.jointControllerClient.FeedbackFcn = [];
            this.jointControllerClient.ResultFcn = [];

            % Robot
            this.UR5eROBOT               = loadrobot("universalUR5e", "DataFormat", "row");
            this.UR5eROBOT               = this.urdfAdjustment(this.UR5eROBOT,"UR5e",0);
            this.initialRobotJConfig     = [0,0,0,0,0,0];

            % IKs
            this.ik                      = inverseKinematics("RigidBodyTree",this.UR5eROBOT);
            this.ik_weights              = [0.25, 0.25, 0.25, 0.1, 0.1, 0.1];
            this.gik_weights             = [0.25, 0.1];
            this.gik                     = generalizedInverseKinematics;
            this.gik.RigidBodyTree       = this.UR5eROBOT;
            this.gik.ConstraintInputs    = {'pose', 'jointbounds'};

            this.jointCon = constraintJointBounds(this.UR5eROBOT);
            this.jointCon.Bounds = [-2.53, -pi/4, -pi/8, -pi+0.05, -pi/4, -pi; ...
                                  2.00,  pi/2, pi-0.3, 0.00,    pi/4, pi]';

            % TF
            this.tftree                  = rostf('DataFormat','struct');
            this.tf_listening_time       = 10;

            % Vision
            this.rgb_sub                 = rossubscriber('/camera/rgb/image_raw','DataFormat','struct');
            this.caminfo_sub             = rossubscriber('/camera/rgb/camera_info','DataFormat','struct');
            this.pt_cloud_sub            = rossubscriber('/camera/depth/points','DataFormat','struct');
            this.objectIdentifier        = ObjectIdentifier(this, this.options);
        end

        function disconnect(this)
            % Shutdown ROS connection
            if this.IsConnected
                rosshutdown;
                this.IsConnected = false;
                cprintf('text', ' - ROS connection closed\n');
            end
        end
    end
end