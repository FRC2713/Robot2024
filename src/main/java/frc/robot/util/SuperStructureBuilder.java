package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import lombok.Builder;
import lombok.Getter;

@Builder
public class SuperStructureBuilder extends SequentialCommandGroup {
  @Getter
  private double elevatorHeight,
      shooterPivotAngleDegrees,
      intakeMotorSpeed,
      feederMotorSPeed,
      shooterMotorSpeed;

  public SuperStructureBuilder run() {
    addCommands(Elevator.Commands.toHeight(this), ShooterPivot.Commands.setTargetAngle(this));
    return this;
  }
}

/*

driver.A().onTrue(SuperStructureBuilder.builder().elevatorHeight(123)....)



*/
