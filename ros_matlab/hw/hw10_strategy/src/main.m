% Main Code to Start ARM Pick and Place.

logPrint(0, 'main', 0, '*blue',  '==========================================================');
logPrint(0, 'main', 0, '*green', '           ARM Pick and Place Challenge Started           ');
logPrint(0, 'main', 0, '*green', '              submission by Aidan Lovelace                ');
logPrint(0, 'main', 0, '*blue',  '==========================================================');
% Create options with the ROS connection
if ~exist('dontreload', 'var')
    options = OptionsConfig;
    logPrint(options.LogVerbosity);
    robotHandle = RobotHandle(options);
    robotHandle.connect();
end


% Go Home
logPrint(0, 'main', 0, 'blue', "Homing Robot");
robotHandle.moveToStdPosition('home', 0.5);

% Reset the simulation
logPrint(0, 'main', 0, 'blue', "Resetting World");
robotHandle.resetWorld();

% Gray zone 1 & 2, easy
logPrint(0, 'main', 0, '*blue', "========= Zones 1 =========");
logPrint(0, 'main', 0, '', "Items will remain the same. No changes to orientation, position, or shape,");
logPrint(0, 'main', 0, '', "so positions are hardcoded. ");
PickandPlaceARMChallenge('static-Zone1', robotHandle);

logPrint(0, 'main', 0, '*blue', "========= Zones 2 =========");
logPrint(0, 'main', 0, '', "Items will remain the same. No changes to orientation, position, or shape,");
logPrint(0, 'main', 0, '', "so positions are hardcoded. ");
PickandPlaceARMChallenge('static-Zone2', robotHandle);

% Yellow zone 3, medium
logPrint(0, 'main', 0, '*blue', "========= Zone 3 =========");
logPrint(0, 'main', 0, '', "Objects in this zone can switch type but will not change position.");
PickandPlaceARMChallenge('Zone3A', robotHandle);
PickandPlaceARMChallenge('Zone3B', robotHandle);

% Red zone 4, hard
logPrint(0, 'main', 0, '*blue', "========= Zone 4 =========");
logPrint(0, 'main', 0, '', "Items will remain in the same position but may change orientation. I.e., rotated");
PickandPlaceARMChallenge('Zone4A', robotHandle);
PickandPlaceARMChallenge('Zone4B', robotHandle);

