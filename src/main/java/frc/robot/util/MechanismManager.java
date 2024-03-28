package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Robot;

public class MechanismManager {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d elevator;
  private final MechanismRoot2d shooterPivot;
  private final MechanismLigament2d shooter;

  public MechanismManager() {
    mech = new Mechanism2d(10, 10);
    root = mech.getRoot("ElevatorBase", 5, 5 - ((double) 1 / 3));

    elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                Constants.ElevatorConstants.MAX_HEIGHT_METERS
                    - (Constants.ElevatorConstants.MIN_HEIGHT_METERS - ((double) 1 / 3)),
                90,
                20,
                new Color8Bit(255, 0, 0)));

    shooterPivot =
        mech.getRoot(
            "ShooterPivot", 5, 5 + Units.inchesToMeters(Robot.elevator.getCurrentHeight()));

    shooter =
        shooterPivot.append(
            new MechanismLigament2d(
                "Shooter",
                0.5,
                Robot.shooterPivot.getCurrentAngle(),
                20,
                new Color8Bit(0, 0, 255)));
    SmartDashboard.putData("Mech2d", mech);
  }

  public void periodic() {
    shooterPivot.setPosition(5, 5 + Units.inchesToMeters(Robot.elevator.getCurrentHeight()));
    shooter.setAngle(Robot.shooterPivot.getCurrentAngle() + 180);
  }
}
