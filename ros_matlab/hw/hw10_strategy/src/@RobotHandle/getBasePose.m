function basePoseRet = getBasePose(this)
    persistent basePose;
    if isempty(basePose)
        basePose = this.getModelPose('robot');
    end
    basePoseRet = basePose;
end