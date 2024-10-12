package frc.robot.util;

import frc.robot.Robot.RobotMode;

public class ModeManager 
{
    private RobotMode mode;
    
    public ModeManager(RobotMode initialMode) {
        this.mode = initialMode;
    }
    
    public RobotMode getMode() {
        return this.mode;
    }

    public void setMode(RobotMode newMode) {
        this.mode = newMode;
    }
}

