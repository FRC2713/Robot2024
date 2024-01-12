import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.FourBarConstants;

class intakePivotIOSim implements IntakePivotIO
{
    private static final SingleJoinedArmSim sim = new SingleJoinedArmSim(
        new SingleJointedArmSim(
        DCMotor.getNEO(1),
        Constants.IntakePivotConstans.GEARING,
        SingleJointedArmSim.estimateMOI(Constants.IntakePivotConstans.LENGTH_METERS,Constants.IntakePivotConstans.MASS_KG),
        Constants.IntakePivotConstans.LENGTH_METERS,
        Units.degreesToRadians(Constants.IntakePivotConstans.MAX_ANGLE_DEGREES - 10),
        Units.degreesToRadians(Constants.IntakePivotConstans.RETRACTED_ANGLE_DEGREES + 10),
        Constants.IntakePivot.SIMULATE_GRAVITY);
    )

    //TODO
    @Override
    public void setCurrentLimit(int currentLimit)
    {

    }

    public void setPosition(double angleDeg)
    {
        sim.setState(VecBuilder.fill(Units.degreesToRadians(angleDeg),0.0))
    }

    @Override
    public void setVoltage(double volts)
    {
        sim.setInputVoltage(volts);
    }
    
    public void reseed(double absoluteEncoderVolts) {
        sim.setState(
            VecBuilder.fill(
                Units.degreesToRadians(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES), 0.0));
      }

}