function testPtCloudImgBBoxOverlap(optns)
    currentFolderContents = dir(pwd);      % Returns all files and folders in the current folder
    currentFolderContents (~[currentFolderContents.isdir]) = [];  % Only keep the folders

    for i = 3:length(currentFolderContents) % Start with 3 to avoid '.' and '..' 
        addpath(['./' currentFolderContents(i).name]) 
    end

    [startZone] = returnZoneJointConfig("Zone1");
    goHome("qr", optns);
    moveToQ("Custom",optns,startZone);
    resetWorld(optns);

    [objects, W_T_ptCloud, ~, ~, W_T_R, R_T_C, frustums] = locateObjectsHere(optns);
    
    debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums);

    for i = 1:size(objects,1)
        obj = objects{i};
        pick("topdown", obj.pickPose,optns);
        moveToQ("Custom",optns, startZone);
        place("topdown", "can", optns); % label for knowing which bin to go to
        moveToQ("Custom", optns, startZone);
    end
end
