%% Main Code to Start ARM Pick and Place.
if not(exist('dontreload','var') && dontreload)
    clear; close all; clc; echo off;
    logPrint(0, 'main', 0, '*blue',  '==========================================================');
    logPrint(0, 'main', 0, '*green', '           ARM Pick and Place Challenge Started           ');
    logPrint(0, 'main', 0, '*green', '              submission by Aidan Lovelace                ');
    logPrint(0, 'main', 0, '*blue',  '==========================================================');
    logPrint(0, 'main', 0, 'cyan', "Preparing workspace...")

    currentFolderContents = dir(pwd);      % Returns all files and folders in the current folder
    currentFolderContents (~[currentFolderContents.isdir]) = [];  % Only keep the folders

    for i = 3:length(currentFolderContents) % Start with 3 to avoid '.' and '..' 
        addpath(['./' currentFolderContents(i).name]);
    end

    optns = initRobotConn();
    global logVerbosity;
    logVerbosity = optns{'logVerbosity'};
    logPrint(0, 'main', 0, 'cyan', "Log Verbosity is " + string(logVerbosity));
else
    logPrint(0, 'main', 0, '*blue',  '==========================================================');
    logPrint(0, 'main', 0, '*green', '           ARM Pick and Place Challenge Started           ');
    logPrint(0, 'main', 0, '*green', '              submission by Aidan Lovelace                ');
    logPrint(0, 'main', 0, '*blue',  '==========================================================');
    logPrint(0, 'main', 0, 'cyan', "Skipping Reload. Using existing workspace.")
    global logVerbosity;
    logVerbosity = optns{'logVerbosity'};
    logPrint(0, 'main', 0, 'cyan', "Log Verbosity is " + string(logVerbosity));
end

%% Go Home 

% Go to home position. TODO: if arm already at home, skip call. 
logPrint(0, 'main', 0, 'blue', "Homing Robot");
goHome('qr', optns);    

%% Reset the simulation

% Reset the world
logPrint(0, 'main', 0, 'blue', "Resetting World");
resetWorld(optns);

% Gray zone 1 & 2, easy 

% logPrint(0, 'main', 0, '*blue', "========= Zones 1 & 2 =========");
% logPrint(0, 'main', 0, '', "Items will remain the same. No changes to orientation, position, or shape,");
% logPrint(0, 'main', 0, '', "so positions can be hardcoded. Currently, we are not hardcoding the positions.");

% % Can create a flag in optns to choose whether to do static/automated.

% PickandPlaceARMChallenge('Zone1', optns);
% PickandPlaceARMChallenge('Zone2', optns);

% % Yellow zone 3, medium
logPrint(0, 'main', 0, '*blue', "========= Zone 3 =========");
logPrint(0, 'main', 0, '', "Objects in this zone can switch type but will not change position.");
PickandPlaceARMChallenge('Zone3', optns);

% % % Red zone 4, hard
% logPrint(0, 'main', 0, '*blue', "========= Zone 4 =========");
% logPrint(0, 'main', 0, '', "Items will remain in the same position but may change orientation. I.e., rotated");
% PickandPlaceARMChallenge('Zone4', optns);

% % % Blue zone 5, very hard
% logPrint(0, 'main', 0, '*blue', "========= Zone 5 =========");
% logPrint(0, 'main', 0, '', "This bin will contain the same items (1 green can, 3 yellow cans and 1 yellow bottle), however the items may change position or orientation.");
% PickandPlaceARMChallenge('Zone5', optns);
