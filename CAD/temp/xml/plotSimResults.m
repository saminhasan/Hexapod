function plotSimResults(simOut, jointAngles, trajectory, sp)
    plotPlatformInfo(simOut, trajectory, sp);
    plotMotorInfo(simOut, jointAngles);
end