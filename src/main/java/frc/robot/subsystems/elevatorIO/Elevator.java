package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;

public class Elevator extends SubsystemBase {
  public final ElevatorIO IO;

  private final ProfiledPIDController elevatorControl;
  private double targetHeight = 0;

  public Elevator(ElevatorIO IO) {
    this.IO = IO;
    elevatorControl = new 
  }

  public void setTargetHeight(double targetHeightInches)
  {
    if(targetHeightInches > Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS))
    {
      RedHawkUtil.ErrHandler.getInstance().addError("Target height too high");
      this.targetHeight = MathUtil.clamp(targetHeightInches, 0, Units.metersToInches(Constants.ElevatorConstants.MAX_HEIGHT_METERS));
      return;
    }
    this.targetHeight = targetHeightInches;
  }
  @Override
  public void periodic()
  {
    double effortLeft = elevatorController.calculate(
            // (inputs.heightInchesLeft + inputs.heightInchesRight) / 2, targetHeight); left encoder
            // returns 0 for a cycle sometimes
            inputs.heightInchesRight, targetHeight)  
  }
}
