package frc.robot.subsystems.feederIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.rhr.RHRPIDFFController;
import frc.robot.util.PIDFFGains;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private FeederIO io;
  private double target;
  private FeederInputsAutoLogged inputs;
  private RHRPIDFFController controller =
      new RHRPIDFFController(PIDFFGains.builder().kP(0.04).kD(0.).build());

  public Feeder(FeederIO io) {
    this.io = io;
    this.inputs = new FeederInputsAutoLogged();
  }

  public void setTarget(double targetRPM) {
    target = targetRPM;
    // io.setSetpoint(targetRPM);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    var volts = controller.calculate(inputs.velocityRPM, target);
    volts = MathUtil.clamp(volts, -12, 12);

    Logger.recordOutput("Feeder/Sent Volts", volts);
    Logger.recordOutput("Feeder/TargetRPM", target);
    Logger.recordOutput("Feeder/atTarget", atTarget());

    io.setVoltage(volts);

    Logger.processInputs("Feeder", inputs);
  }

  public boolean atTarget() {
    return Math.abs(inputs.velocityRPM - target) < 0.1;
  }

  public static class Commands {
    public static Command setToVelocity(double rpm) {
      return new InstantCommand(() -> Robot.feeder.setTarget(rpm));
    }

    public static Command setToVelocityAndWait(double rpm) {
      return new SequentialCommandGroup(
          setToVelocity(rpm), new WaitUntilCommand(Robot.feeder::atTarget));
    }
  }
}
