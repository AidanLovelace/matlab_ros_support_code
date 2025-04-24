function merged_ptCloud = threadedMergePtClouds(ptCloud1, ptCloud2, table_roi)
    merged_ptCloud = pcmerge(ptCloud1, ptCloud2, 0.001);
    % Filter to just the table ROI
    indices = findPointsInROI(merged_ptCloud, table_roi);
    merged_ptCloud = select(merged_ptCloud, indices);
end