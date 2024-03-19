package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private int numLed = 64;
  private double sparking = 0;
  private double cooling = 0;
  private boolean reverseDirection;
  private int ledOffset = 0;

  public Candle(boolean isSimulation) {
    this.m_candle = isSimulation ? new CandleSim() : new CANdle(Constants.RobotMap.CANDLE_CAN_ID);
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
    m_candle.setLEDs(this.r, this.g, this.b);
  }

  public void changeAnimation(AnimationTypes led_animation) {
    switch (led_animation) {
      case ColorFlow:
        m_animation = new ColorFlowAnimation(r, g, b);
        break;
      case Fire:
        m_animation =
            new FireAnimation(
                brightness, speed, numLed, sparking, cooling, reverseDirection, ledOffset);
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
    m_candle.animate(m_animation);
  }

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

  public void setSpCoRdLo(
      double sparking, double cooling, boolean reverseDirection, int ledOffset) {
    this.sparking = sparking;
    this.cooling = cooling;
    this.reverseDirection = reverseDirection;
    this.ledOffset = ledOffset;
  }

  public static class Commands {
    public static Command changeAnimation(AnimationTypes type) {
      return new InstantCommand(() -> Robot.candle.changeAnimation(type));
    }

    public static Command hasGamePieceAnimation(boolean hasGamePiece) {
      if (hasGamePiece) {
        return new SequentialCommandGroup(
            blinkLEDs(0, 255, 0), new WaitCommand(2), setLEDs(0, 255, 0));

      } else {
        return new SequentialCommandGroup(setLEDs(0, 0, 0));
      }
    }

    public static Command shootingAnimation() {
      return new SequentialCommandGroup(
          changeAnimation(AnimationTypes.Larson), new WaitCommand(2), setLEDs(0, 0, 0));
    }

    public static Command setLEDs(int r, int g, int b) {
      return new InstantCommand(
          () -> {
            Robot.candle.setRGBValue(r, g, b);
            Robot.candle.changeAnimation(AnimationTypes.SetAll);
          });
    }

    public static Command blinkLEDs(int r, int g, int b) {
      return new InstantCommand(
          () -> {
            Robot.candle.setRGBValue(r, g, b);
            Robot.candle.changeAnimation(AnimationTypes.Strobe);
          });
    }

    public static Command LEDsOff() {
      return Candle.Commands.setLEDs(0, 0, 0);
    }

    public static Command gamePieceDetected() {
      return new InstantCommand(
          () -> {
            Robot.candle.setRGBValue(255, 165, 0);
            Robot.candle.changeAnimation(AnimationTypes.SetAll);
          });
    }

    public static Command gamePieceLockedOn() {
      return new InstantCommand(
          () -> {
            Robot.candle.setRGBValue(255, 165, 0);
            Robot.candle.changeAnimation(AnimationTypes.Strobe);
          });
    }
  }
}
