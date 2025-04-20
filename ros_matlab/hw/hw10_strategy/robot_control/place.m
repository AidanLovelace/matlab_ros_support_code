function grip_result = place(strategy,label,optns)
    %--------------------------------------------------------------------------
    
    %--------------------------------------------------------------------------
    
        %greenBin = [2.2 0 pi/2 -pi/2 0 0];
        greenBin = [5/8*pi 0 pi/2.3 -pi/2.3 0 0];
        blueBin = [-0.8*pi 0 pi/2.3 -pi/2.3 0 0];
        
        %% 2. Move to desired location
        if strcmpi(strategy,'topdown')
            logPrint(4, 'place-'+string(label), 3, '', "Topdown Strategy");
            
            %% For Blue Bin Objects -- bottles (vertical or horizontal) / Markers/ Blue and Red Cubes k
            if contains(label, 'ottle') || contains(label, 'marker') || contains(label, 'pouch')
                logPrint(4, 'place-'+string(label), 3, '', "Target: Blue Bin");
                
                % Move to BlueBin
                logPrint(4, 'place-'+string(label), 3, '', "Moving to Blue Bin");
                moveToQ("custom",optns,blueBin);
                
                % Release
                logPrint(4, 'place-'+string(label), 3, '', "Opening Gripper");
                [grip_result,grip_state] = doGrip('place',optns); 
                grip_result = grip_result.ErrorCode;
                
            else
                %% For Green Bin Objects -- cans/spam/Green and Purple cubes
                logPrint(4, 'place-'+string(label), 3, '', "Target: Green Bin");
                logPrint(4, 'place-'+string(label), 3, '', "Moving to Green Bin");
                moveToQ("custom",optns,greenBin);
                logPrint(4, 'place-'+string(label), 3, '', "Opening Gripper");
                [grip_result,grip_state] = doGrip('place',optns); 
                grip_result = grip_result.ErrorCode;
            end
        end
    end
    