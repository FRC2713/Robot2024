package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO{

    private static final FlywheelSim leftFlyWheel = new FlywheelSim(DCMotor.getNeoVortex(1),Constants.ShooterConstants.GEARING,0.001);

    private static final FlywheelSim rightFlyWheel = new FlywheelSim(DCMotor.getNeoVortex(1),Constants.ShooterConstants.GEARING,0.001);

    @Override
    public void updateInputs(ShooterInputs inputs) {
        //setLeftVoltage(MathUtil.clamp(0, 0, 0));
        leftFlyWheel.update(0.02);
        rightFlyWheel.update(0.02);
    }

    @Override
    public void setLeftVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setLeftVoltage'");
    }

    @Override
    public void setRightVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRightVoltage'");
    }

    @Override
    public void setLeftMotorRPMSetPoint(double rPM) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setLeftMotorRPMSetPoint'");
    }

    @Override
    public void setRightMotorRPMSetPoint(double rPM) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRightMotorRPMSetPoint'");
    }

}
