package frc.robot.subsystems.shooterIO;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggableMotor;

public class Shooter extends SubsystemBase{

    private final ShooterIO IO;
    private final ShooterInputsAutoLogged inputs;
    private double leftFlyWheelTargetRPM = 0.0;
    private double rightFlyWheelTargetRPM = 0.0;
    private LoggableMotor leftMotor = new LoggableMotor("LeftMotor", DCMotor.getNeoVortex(1));
    private LoggableMotor rightMotor = new LoggableMotor("rightMotor", DCMotor.getNeoVortex(1));
    private ProfiledPIDController leftMotorController;
    private ProfiledPIDController rightMotorController;
    public Shooter(ShooterIO IO)
    {
        this.IO = IO;
        inputs = new ShooterInputsAutoLogged();
        this.IO.updateInputs(inputs);
    }

    @Override
    public void periodic()
    {
        IO.updateInputs(inputs);
    }

    public void setLeftFlyWheelTargetRPM(double leftFlyWheelTargetRPM) {
        this.leftFlyWheelTargetRPM = leftFlyWheelTargetRPM;
    }
    public void setRightFLyWheelTargetRPM(double rightTargetRPM)
    {
        this.rightFlyWheelTargetRPM = rightFlyWheelTargetRPM;
    }

}
