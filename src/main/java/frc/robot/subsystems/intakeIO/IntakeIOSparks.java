package frc.robot.subsystems.intakeIO;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOSparks implements IntakeIO {

  private CANSparkMax bottomMotor, topMotor;

  public IntakeIOSparks() {
    this.bottomMotor =
        new CANSparkMax(Constants.RobotMap.INTAKE_BOTTOM_MOTOR_CAN_ID, MotorType.kBrushless);
    this.topMotor =
        new CANSparkMax(Constants.RobotMap.INTAKE_TOP_MOTOR_CAN_ID, MotorType.kBrushless);

    bottomMotor.restoreFactoryDefaults();
    topMotor.restoreFactoryDefaults();

    bottomMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    bottomMotor.setSmartCurrentLimit(40);
    topMotor.setSmartCurrentLimit(40);

    topMotor.setInverted(false);
    bottomMotor.setInverted(false);
  }

  @Override
  public void updateInputs(IntakeInputsAutoLogged inputs) {
    inputs.leftOutputVoltage = MathUtil.clamp(bottomMotor.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.leftIsOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(bottomMotor.getEncoder().getVelocity()))
            > 0.005;
    inputs.leftVelocityRPM = bottomMotor.getEncoder().getVelocity();
    inputs.leftTempCelcius = bottomMotor.getMotorTemperature();
    inputs.leftCurrentAmps = bottomMotor.getOutputCurrent();
    inputs.leftPositionRad = bottomMotor.getEncoder().getPosition() * Math.PI * 2;

    inputs.rightOutputVoltage = MathUtil.clamp(topMotor.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.rightIsOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(topMotor.getEncoder().getVelocity()))
            > 0.005;
    inputs.rightVelocityRPM = topMotor.getEncoder().getVelocity();
    inputs.rightTempCelcius = topMotor.getMotorTemperature();
    inputs.rightCurrentAmps = topMotor.getOutputCurrent();
    inputs.rightPositionRad = bottomMotor.getEncoder().getPosition() * Math.PI * 2;
  }

  @Override
  public void setVoltage(double bottomVolts, double topVolts) {
    bottomMotor.setVoltage(bottomVolts);
    topMotor.setVoltage(topVolts);
  }
}
