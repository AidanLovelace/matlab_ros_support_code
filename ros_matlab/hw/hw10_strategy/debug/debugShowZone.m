function debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums)
    figure;
    pcshow(W_T_ptCloud);
    hold on;
    for i = 1:size(objects,1)
        obj = objects{i};
        % cprintf('text', ' - %s - (%.2f, %.2f, %.2f)...\n', obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));
        % Plot the fitted cylinder
        plot(obj.model);
        hold on;
        % Plot the top center point of the cylinder
        scatter3(obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3), 150, [1 0 1], 'filled');
        text(obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3), obj.label, "Color", [1 1 1], "FontWeight", "bold", "HorizontalAlignment", "right", "VerticalAlignment", "bottom");
        hold on;
    end

    % Get positions of points of interest
    wtr = [W_T_R.Pose.Position.X;W_T_R.Pose.Position.Y;W_T_R.Pose.Position.Z];
    rtc = [R_T_C.Transform.Translation.X;R_T_C.Transform.Translation.Y;R_T_C.Transform.Translation.Z];
    wtc = wtr + rtc;

    u = [wtr(1) wtc(1)];
    v = [wtr(2) wtc(2)];
    Z = [wtr(3) wtc(3)];

    axis on;
    xlabel("X"); ylabel("Y"); zlabel("Z");
    hold on;
    scatter3(u, v, Z, 100, [1 0 0; 1 0 0], 'filled');
    text(u, v, Z, ["Robot Base", "Camera"], "Color", [1 1 1], "FontWeight", "bold", "HorizontalAlignment", "center", "VerticalAlignment", "bottom");
    hold on;
    visualizeFrustum(frustums);
    title('Debug Display: Whole Zone'); hold off;
end