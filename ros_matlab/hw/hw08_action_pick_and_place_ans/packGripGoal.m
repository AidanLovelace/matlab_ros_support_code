function gripGoal=packGripGoal(pos,gripGoal)
    %----------------------------------------------------------------------
    % packGripGoal
    %----------------------------------------------------------------------
    % This function will populate gripGoal with all information necessary
    % in a control_msgs/FollowJointTrajectoryGoal message.
    %
    % This functions assumes a single waypoint for the gripper, i.e. open
    % or close. 
    %
    % The function will accomplish 3 steps:
    % 1) Set Point, name and duration. 
    % 2) Set Goal Tolerance via a control_msgs/JointTolerance message type
    % 3) Set a trajectory_msgs/JointTrajectoryPoint with time/pos/vel/accel/effort
    %
    % Input Arguments:
    % pos (double): a position where 0 is open and 
    % gripGoal (control_msgs/FollowJointTrajectoryGoal)
    %
    % Output
    % A populated gripGoal message
    %----------------------------------------------------------------------
    
    r = rosClassHandle;

    % 1. Set Point, name and Duration
    jointWaypoints = pos;                % Set position of way points
    jointWaypointTimes = 0.01;             % Time at which action is launched
    numJoints = size(jointWaypoints,1);    % Only 1 joint for gripper

    % Joint Names --> gripGoal.Trajectory
    gripGoal.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};

    % 2. Set Goal Tolerance: set type, name, and pos/vel/acc tolerance
    % Note: tolerances are considered as: waypoint +/- tolerance
    % gripGoal.GoalTolerance = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');
    % 
    % gripGoal.GoalTolerance.Name         = gripGoal.Trajectory.JointNames{1};
    % gripGoal.GoalTolerance.Position     = 0;
    % gripGoal.GoalTolerance.Velocity     = 0.1;
    % gripGoal.GoalTolerance.Acceleration = 0.1;
    
    %3. Create a trajectory_msgs/JointTrajectoryPoint. Copy TimeFromSTart
    % and Positions in here along with vel/acc/effort. Finally copy this
    % variable into grip.Goal.Trajectory.Points
    
    % Time Stamp
    if numJoints == 1
        r.trajPtsVar.TimeFromStart   = rosduration(1);
    else
        r.trajPtsVar.TimeFromStart   = rosduration(jointWaypointTimes);
    end
    
    % Position
    r.trajPtsVar.Positions       = jointWaypoints;

    % Vel/Acc/Effort
    r.trajPtsVar.Velocities      = zeros(size(jointWaypoints));
    r.trajPtsVar.Accelerations   = zeros(size(jointWaypoints));
    r.trajPtsVar.Effort          = zeros(size(jointWaypoints));
    
    % Copy trajPts --> gripGoal.Trajectory.Points
    gripGoal.Trajectory.Points = r.trajPtsVar;
end