%% Main Code to Start ARM Pick and Place.
clear; close all; clc; echo off;

cprintf('*blue', '\n===========================================\n');
cprintf('*green', '   ARM Pick and Place Challenge Started\n');
cprintf('*green', '      submission by Aidan Lovelace\n');
cprintf('*blue', '===========================================\n');
cprintf('cyan', '\nPreparing workspace...\n');

optns = initRobotConn();

%% Go Home 

% Go to home position. TODO: if arm already at home, skip call. 
cprintf('blue','Moving robot to home position...\n');
goHome('qr', optns);    

%% Reset the simulation

% Reset the world
cprintf('blue','Resetting the world...\n');
resetWorld(optns);

% Gray zone 1 & 2, easy 

cprintf('blue','Starting Static Zones 1 & 2...\n');

% Can create a flag in optns to choose whether to do static/automated.
optns{'static'} = false;

if optns{'static'}
    cprintf('text',' - Using static positions\n');
    % -- can set things statically (needs to identiy poses if scene changes)
    staticPickAndPlace(optns); % Not preferred. 

else
    % Automated method
    cprintf('text',' - Using pose estimation\n');
    PickandPlaceARMChallenge('Zone1', optns);
    PickandPlaceARMChallenge('Zone2', optns);
end

% Yellow zone 3, medium
cprintf('blue','\n\nStarting Zone 3...\n');
PickandPlaceARMChallenge('Zone3', optns);

% Red zone 4, hard
cprintf('blue','\n\nStarting Zone 4...\n');
PickandPlaceARMChallenge('Zone4', optns);

% Blue zone 5, very hard
cprintf('blue','\n\nStarting Zone 5...\n');
PickandPlaceARMChallenge('Zone5', optns);
