package frc.robot.subsystems.intakeIO;

import java.util.regex.Pattern;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LoggableMotor;

public class Intake extends SubsystemBase
{
    private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  private double targetRPM = 0.0;
  private double cubeDetectionThreshold = 0.4;
  private double coneDetectionThreshold = 0.25;
  private double filteredVoltageCube = 0, filteredVoltageCone;
  public boolean scoring = false;
  private boolean previouslyHadGamePiece = false;

  private Timer timer = new Timer();
  //private Debouncer debouncer = new Debouncer(0.1);
  //private LinearFilter analogVoltageFilterRight = LinearFilter.singlePoleIIR(0.04, 0.02);
  //private LinearFilter analogVoltageFilterLeft = LinearFilter.singlePoleIIR(0.04, 0.02);
  private LoggableMotor motor;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
    motor = new LoggableMotor("Intake Roller", DCMotor.getNeo550(1));
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.velocityRPM - targetRPM) < 0.5;
  }

  public void setRPM(double rpm) {
    Logger.recordOutput("Intake/Applied Volts", rpm / (Constants.IntakeConstants.MAX_RPM) * 12);
    IO.setBottomVoltage(rpm / (Constants.IntakeConstants.MAX_RPM) * 12); // PLACEHOLDER VALUE
  }

  public double getCurrentDraw() {
    return inputs.currentAmps;
  }

  public boolean hasGamepiece() {
    //if (Robot.gamePieceMode == GamePieceMode.CUBE) {
    return IO.hasGamepiece();
    //} else {
      //return debouncer.calculate(filteredVoltageCone > coneDetectionThreshold) && !scoring;
    //}
  }

  public void setScoring(boolean scoring) {
    this.scoring = scoring;
  }

  public void periodic() {
    IO.updateInputs(inputs);
    motor.log(inputs.currentAmps, inputs.outputVoltage);
    //bottomRollerMotor.log(inputs.bottomCurrentAmps, inputs.bottomOutputVoltage);


    Logger.recordOutput("Intake/Sensor Range",this.inputs.sensorRange);
    //Logger.recordOutput("Intake/Filtered Sensor Cone", filteredVoltageCone);

    Logger.recordOutput("Intake/Target RPM", targetRPM);
    Logger.recordOutput("Intake/Has reached target", isAtTarget());

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Scoring", scoring);
    Logger.recordOutput("Intake/Has gamepiece", hasGamepiece());


    /*if (hasGamepiece() && !previouslyHadGamePiece) {
      timer.restart();
      previouslyHadGamePiece = true;
      // RumbleManager.getInstance().setDriver(1.0, 2.0);
      if (timer.get() <= 2.0) {
        Robot.lights.setColorPattern(Pattern.DarkGreen);
      }
    }*/

    if (!hasGamepiece()) {
        //is this a troll?
      previouslyHadGamePiece = !true;
    }
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {

    public static Command setVelocityRPM(double targetRPM)
    {
        return new InstantCommand(()-> Robot.intake.setRPM(targetRPM));
    }

    public static Command setTopVelocityRPM(double targetRPM) {
      return setVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }
  }    
}
