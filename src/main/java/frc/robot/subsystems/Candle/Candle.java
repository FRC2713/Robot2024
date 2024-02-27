package frc.robot.subsystems.Candle;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Candle extends SubsystemBase {

  private CANdle m_candle = new CANdle(Constants.RobotMap.CANDLE_CAN_ID);
  private Animation m_animation = null;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  @Override
  public void periodic() {
    m_candle.animate(m_animation);
  }

  public void changeAnimation(AnimationTypes led_animation) {
    switch (led_animation) {
      case ColorFlow:
        m_animation = new ColorFlowAnimation(0, 0, 0);
        break;
      case Fire:
        m_animation = new FireAnimation();
        break;
      case Larson:
        m_animation = new LarsonAnimation(0, 0, 0);
        break;
      case Rainbow:
        m_animation = new RainbowAnimation(0, 0, 0);
        break;
      case RgbFade:
        m_animation = new RgbFadeAnimation(0, 0, 0);
        break;
      case SingleFade:
        m_animation = new SingleFadeAnimation(0, 0, 0);
        break;
      case Strobe:
        m_animation = new StrobeAnimation(0, 0, 0);
        break;
      case Twinkle:
        m_animation = new TwinkleAnimation(0, 0, 0);
        break;
      case TwinkleOff:
        m_animation = new TwinkleOffAnimation(0, 0, 0);
        break;
      case SetAll:
        m_animation = null;
        break;
    }
  }

  public static class Commands {
    public static Command changeAnimation(AnimationTypes type) {
      return new InstantCommand(() -> Robot.candle.changeAnimation(type));
    }
  }
}
