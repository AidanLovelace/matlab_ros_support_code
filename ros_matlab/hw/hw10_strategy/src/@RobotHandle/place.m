function grip_result = place(this, object)

        greenBin = [5/8*pi 0 pi/2.3+0.1 -pi/2.3-0.1 0 0];
        blueBin = [-0.8*pi 0 pi/2.3+0.1 -pi/2.3-0.1 0 0];

        blueBinPose = [1.0000         0         0    0.3957;...
                            0   -1.0000   -0.0000   -0.3179;...
                            0    0.0000   -1.0000    0.5289;...
                            0         0         0    1.0000];
        greenBinPose = [1.0000         0         0   -0.4014;...
                            0   -1.0000   -0.0000   -0.3106;...
                            0    0.0000   -1.0000    0.5290;...
                            0         0         0    1.0000];

        label = object.label;

        % For Blue Bin Objects -- bottles (vertical or horizontal) / Markers/ Blue and Red Cubes k
        if contains(label, 'ottle') || contains(label, 'marker') || contains(label, 'pouch')
            logPrint(4, 'place-'+string(label), 3, '', "Target: Blue Bin");

            % Move to BlueBin
            logPrint(4, 'place-'+string(label), 3, '', "Moving to Blue Bin");
            this.moveToPose(blueBinPose, 1.5);
            this.moveToPose(blueBinPose, 0.5);
            pause(1);
            this.moveToPose(lift(blueBinPose, -0.4), 0.5);
            this.moveToPose(lift(blueBinPose, -0.6), 0.5);

            pause(1);

            % Release
            logPrint(4, 'place-'+string(label), 3, '', "Opening Gripper");
            [grip_result, ~] = this.setGripper(0);
            pause(0.5);
            [grip_result, ~] = this.setGripper(1);
            pause(0.5);
            [grip_result, ~] = this.setGripper(0);
            grip_result = grip_result.ErrorCode;
            pause(1);
            this.moveToPose(blueBinPose, 0.5);

        else
            % For Green Bin Objects -- cans/spam/Green and Purple cubes
            logPrint(4, 'place-'+string(label), 3, '', "Target: Green Bin");
            logPrint(4, 'place-'+string(label), 3, '', "Moving to Green Bin");
            this.moveToPose(greenBinPose, 1.5);
            this.moveToPose(greenBinPose, 0.5);
            this.moveToPose(lift(greenBinPose, -0.4), 0.5);
            this.moveToPose(lift(greenBinPose, -0.5), 0.5);

            pause(1);

            logPrint(4, 'place-'+string(label), 3, '', "Opening Gripper");
            [grip_result, ~] = this.setGripper(0);
            pause(0.5);
            [grip_result, ~] = this.setGripper(1);
            pause(0.5);
            [grip_result, ~] = this.setGripper(0);
            grip_result = grip_result.ErrorCode;
            pause(1);
            this.moveToPose(greenBinPose, 0.5);
        end
    end
