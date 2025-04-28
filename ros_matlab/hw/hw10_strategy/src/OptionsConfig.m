classdef OptionsConfig < handle
    properties
        % Debug and logging options
        Debug = 0
        LogVerbosity = 6

        % Tool options
        ToolFlag = 0
        ToolAdjustmentFlag = 1
        ToolAdjustment = 0.165

        % Trajectory options
        TrajSteps = 1
        TrajDuration = 0.5

        % Position offsets
        XOffset = 0
        YOffset = 0
        ZOffset = 0.2

        GripPos = 0.23

        % Frame options
        FrameAdjustmentFlag = 1

        % Simulation options
        CleanStart = true
        UseZoneObjectsCache = false
        ComparePicksWithGazebo = true
    end

    methods

    end
end