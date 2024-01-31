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
  // private Vector elevatorPosition;
  public MechanismManager() {
    // shooterPivot = null;
    mech = new Mechanism2d(10, 10);
    // elevatorPosition = new Vector(5, 5);
    root = mech.getRoot("ElevatorBase", 5, 5);

    elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                Constants.ElevatorConstants.MAX_HEIGHT_METERS
                    - Constants.ElevatorConstants.MIN_HEIGHT_METERS,
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
    shooter.setAngle(Robot.shooterPivot.getCurrentAngle());
  }
}

/*class Vector {
    @ Getter private double x;
    @ Getter private double y;

    Vector(double x,double y)
    {
        this.x = x;
        this.y = y;
    }

    public void add(Vector vec)
    {
        this.x+=vec.x;
        this.y+=vec.x;
    }

    public void sub(Vector vec)
    {
        this.x-=vec.x;
        this.y-=vec.x;
    }

    public static Vector staticAdd(Vector vec1, Vector vec2)
    {
        return new Vector(vec1.getX()+vec2.getX(), vec1.getY()+vec2.getY());
    }
    public static Vector staticSub(Vector vec1,Vector vec2)
    {
        return new Vector(vec1.getX()-vec2.getX(),vec1.getY()-vec2.getY());
    }
}*/
