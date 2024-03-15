package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Candle extends SubsystemBase {

  private CANdle m_candle;
  private Animation m_animation = null;

  private int r = 0;
  private int g = 0;
  private int b = 0;
  private double brightness = 0;
  private double speed = 0;
  private int numLed = 0;
  private double sparking = 0;
  private double cooling = 0;
  private boolean reverseDirection;
  private int ledOffset = 0;

  public Candle(boolean isSimulation) {
    this.m_candle = isSimulation ? null : new CANdle(Constants.RobotMap.CANDLE_CAN_ID);
  }

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
    if (m_candle == null) {
      // do nothing
    } else if (m_animation == null) {
      m_candle.setLEDs(this.r, this.g, this.b);
    } else {
      m_candle.animate(m_animation);
    }
  }

  public void changeAnimation(AnimationTypes led_animation) {

    // TODO: for relavant animations, use the r, g, and b class variabls instead of the hard coded ones

    switch (led_animation) {
      case ColorFlow:
        m_animation = new ColorFlowAnimation(r, g, b);
        break;
      case Fire:
        m_animation = new FireAnimation(brightness, speed, numLed, sparking, cooling, reverseDirection, ledOffset);
        break;
      case Larson:
        m_animation = new LarsonAnimation(r, g, b);
        break;
      case Rainbow:
        m_animation = new RainbowAnimation(brightness, speed, numLed);
        break;
      case RgbFade:
        m_animation = new RgbFadeAnimation(brightness, speed, numLed);
        break;
      case SingleFade:
        m_animation = new SingleFadeAnimation(r, g, b);
        break;
      case Strobe:
        m_animation = new StrobeAnimation(r, g, b);
        break;
      case Twinkle:
        m_animation = new TwinkleAnimation(r, g, b);
        break;
      case TwinkleOff:
        m_animation = new TwinkleOffAnimation(r, g, b);
        break;
      case SetAll:
        m_animation = null;
        break;
    }
  }

  // TODO: with a method, set r, g, and b
    public void setRGBValue(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
    public void setBrSpNl(double brightness, double speed, int numLed) {
      this.brightness = brightness;
      this.speed = speed;
      this.numLed = numLed;
    }
    public void setSpCoRdLo(double sparking, double cooling, boolean reverseDirection, int ledOffset) {
      this.sparking = sparking;
      this.cooling = cooling;
      this.reverseDirection = reverseDirection;
      this.ledOffset = ledOffset;
    }
  public static class Commands {
    public static Command changeAnimation(AnimationTypes type) {
      return new InstantCommand(() -> Robot.candle.changeAnimation(type));
    }
  }
}
