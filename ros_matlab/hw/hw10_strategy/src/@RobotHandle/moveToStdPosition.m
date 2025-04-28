function result = moveToStdPosition(this, stdPosition, duration)
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

    switch stdPosition
        case 'home'
            jointAngles = [0 0 pi/2 -pi/2 0 0];
        case 'static'
            jointAngles = [0 0 pi/2 -pi/2 0 0];
        case 'qr'
            jointAngles = [0 0 pi/2 -pi/2 0 0];
        case 'zero'
            jointAngles = zeros(1,6);
        case 'qz'
            jointAngles = zeros(1,6);
        case 'qtest'
            jointAngles = [0 pi/4 pi/4 -pi/2 0 0];
        case 'Zone1A'
            jointAngles = [1.4,   0.3,    0.9,   -1.2,   0,    1.4];
        case 'static-Zone1'
            jointAngles = [1.4,   0.3,    0.9,   -1.2,   0,    1.4];
        case 'Zone1Bottle'
            jointAngles = [1.4,   0.3,    1.2,   -1.5,   0,    1.4];
        case 'Zone1B'
            jointAngles = [1.0,   0.4,    0.9,   -1.3,   0,    1.0];
        case 'Zone1BBottle'
            jointAngles = [1.0,   0.4,    1.1,   -1.5,   0,    1.0];
        case 'static-Zone2'
            jointAngles = [-1.9,   0.1,    1.4,   -1.5,    0,     -1.9];
        case 'Zone3A'
            jointAngles = [-0.4400,   -0.7014,    2.0136,   -1.3122,    0.0002,   -0.4400];
        case 'Zone3B'
            jointAngles = [-0.4400,   -0.7014+.25,    2.0136,   -1.3122-.25,    0.0002,   -0.4400];
        case 'Zone2A'
            jointAngles = [-1.9,   0.1,    1.4,   -1.5,    0,     -1.9];
        case 'Zone4A'
            jointAngles = [-0.1453,     0.4926, 0.8193,     -1.3119,    0.0,        -0.1453];
        case 'Zone4B'
            jointAngles = [-0.1865,    0.1104,    1.7119,   -1.8223,   -0.0000,     -0.1865];
        case 'Zone5A'
            jointAngles =  [-0.9376,    0.1748,    1.4563,   -1.6311,    0.0001,    0.6332];
        otherwise
            error('Invalid stdPosition. Use qr, qz, or qtest.');
    end

    logPrint(8, 'moveToStdPosition', 2, '', "jointAngles (%s) = [%.2f %.2f %.2f %.2f %.2f %.2f]", stdPosition, jointAngles(1),jointAngles(2),jointAngles(3),jointAngles(4),jointAngles(5),jointAngles(6));

    result = this.setJointAngles(jointAngles, duration);
end