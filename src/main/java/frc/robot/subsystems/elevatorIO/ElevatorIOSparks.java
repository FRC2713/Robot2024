package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkMax;

public class ElevatorIOSparks implements ElevatorIO{
    public ElevatorIOSparks() {
    left = new CANSparkMax(Constants.RobotMap.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
    right = new CANSparkMax("left elevator CAN id", MotorType.kBrushless);

    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();
    }


    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.outputVoltageLeft = 0.0;
        inputs.heightInchesLeft = 0.0;
        inputs.velocityInchesPerSecondLeft = 0.0;
        inputs.tempCelsiusLeft = 0.0;
        inputs.currentDrawAmpsLeft = 0.0;
        inputs.outputVoltageRight = 0.0;
        inputs.heightInchesRight = 0.0;
        inputs.velocityInchesPerSecondRight = 0.0;
        inputs.tempCelsiusRight = 0.0;
        inputs.currentDrawAmpsRight = 0.0;
    }

    @Override
    public void resetEncoders() {
        left.getEncoder().setPosition(0);
        right.getEncoder().setPosition(0);;
    }

    @Override
    public boolean shouldApplyFF() {
        return true;
    }

    @Override
    public void setVoltage(double volts) {
        left.setVoltage(volts);
        right.setVoltage(volts);;
    }
    
}
