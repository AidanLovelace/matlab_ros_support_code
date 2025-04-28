function result = moveToStdPose(this, stdPose, duration)
    % moveToQ
    % Move to joint configuration prescribed by config
    %
    % Expansion/TODO:
    % - Check if robot already at desired position. Then skip action calls.
    %
    % Inputs
    % config (string):    home, qr, qz, qtest
    %
    % Outputs:
    % res (bool): 0 indicates success, other failure.
    %----------------------------------------------------------------------

    if nargin < 3
        duration = 0.5;
    end

    switch stdPose
        case 'home'
            pose = [1.0000         0         0   -0.1333;
                         0   -1.0000   -0.0000    0.4919;
                         0    0.0000   -1.0000    0.4879;
                         0         0         0    1.0000];
        case 'qr'
            pose = [1.0000         0         0   -0.1333;
                         0   -1.0000   -0.0000    0.4919;
                         0    0.0000   -1.0000    0.4879;
                         0         0         0    1.0000];
        case 'zero'
            pose = [1.0000         0         0   -0.1333;
                         0   -1.0000   -0.0000    0.0997;
                         0    0.0000   -1.0000    0.8801;
                         0         0         0    1.0000];
        case 'qz'
            pose = [1.0000         0         0   -0.1333;
                         0   -1.0000   -0.0000    0.0997;
                         0    0.0000   -1.0000    0.8801;
                         0         0         0    1.0000];
        case 'Zone1A'
            pose = [1.0000         0         0   -0.5990;
                         0   -1.0000   -0.0000    0.0899;
                         0    0.0000   -1.0000    0.6110;
                         0         0         0    1.0000];
        case 'Zone2A'
            pose = [1.0000         0         0    0.5496;
                         0   -1.0000   -0.0000   -0.0123;
                         0    0.0000   -1.0000    0.5135;
                         0         0         0    1.0000];
        case 'Zone3A'
            pose = [1.0000         0         0   -0.0335;
                         0   -1.0000   -0.0000    0.2419;
                         0    0.0000   -1.0000    0.4879;
                         0         0         0    1.0000];
        case 'Zone4A'
            pose = [1.0000         0         0   -0.0335;
                         0   -1.0000   -0.0000    0.6920;
                         0    0.0000   -1.0000    0.5378;
                         0         0         0    1.0000];
        case 'Zone5A'
            pose = [1.0000         0         0    0.3767;
                         0   -1.0000   -0.0000    0.4419;
                         0    0.0000   -1.0000    0.4578;
                         0         0         0    1.0000];
        otherwise
            error('Invalid stdPosition. Use qr, qz, or qtest.');
    end

    % logPrint(8, 'moveToStdPosition', 2, '', "jointAngles (%s) = [%.2f %.2f %.2f %.2f %.2f %.2f]", stdPose, jointAngles(1),jointAngles(2),jointAngles(3),jointAngles(4),jointAngles(5),jointAngles(6));

    result = this.moveToPose(pose, duration, 1);
end