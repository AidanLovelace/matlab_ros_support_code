function visualizePositions(W_T_R, W_T_M, R_T_C, R_T_ptCloud)
    wtr = [W_T_R.Pose.Position.X;W_T_R.Pose.Position.Y;W_T_R.Pose.Position.Z];
    wtm = [W_T_M.Pose.Position.X;W_T_M.Pose.Position.Y;W_T_M.Pose.Position.Z];
    rtc = [R_T_C.Transform.Translation.X;R_T_C.Transform.Translation.Y;R_T_C.Transform.Translation.Z];
    wtc = wtr + rtc;

    u = [wtr(1) wtm(1) wtc(1)];
    v = [wtr(2) wtm(2) wtc(2)];
    Z = [wtr(3) wtm(3) wtc(3)];

    W_T_ptCloud = pctransform(R_T_ptCloud,rigidtform3d(ros2MLPose(W_T_R)));

    pts = W_T_ptCloud.Location;
    ptsFlat = reshape(pts, [], 3); X2 = ptsFlat(1:6:end,1); Y2 = ptsFlat(1:6:end,2); Z2 = ptsFlat(1:6:end,3);

    

    % figure; scatter3(X2, Y2, Z2, 5, Z2, 'filled');
    figure,pcshow(W_T_ptCloud);axis on;
    xlabel("X"); ylabel("Y"); zlabel("Z");
    hold on;
    scatter3(u, v, Z, 100, [1 0 1; 0 1 0; 0 0 1], 'filled');
    xlabel('X') 
    ylabel('Y') 
    title('Projected Point Cloud on Image (depth coloring)'); hold off;
end

