package frc.robot.subsystems.candle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotMap;
import frc.robot.Robot;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class NewCandle {
  /**
   * Tree map of light code enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, Pair<Integer[], Animation>> lightOptionsMap;

  private CANdle candle;
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    OFF,
    SEES_NOTE,
    LOCKED_ON_NOTE,
    HAS_NOTE,
    RESTING_RED,
    LARSON,
    TWINKLE
  }

  Integer[] white = new Integer[] {255, 255, 255};

  public NewCandle(boolean isSimulation) {
    candle = isSimulation ? new CandleSim() : new CANdle(RobotMap.CANDLE_CAN_ID);

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    lightOptionsMap = new TreeMap<LightCode, Pair<Integer[], Animation>>();
    lightOptionsMap.put(LightCode.OFF, new Pair<>(new Integer[] {0, 0, 0}, null));
    lightOptionsMap.put(LightCode.HAS_NOTE, new Pair<>(new Integer[] {0, 255, 0}, null));
    lightOptionsMap.put(LightCode.SEES_NOTE, new Pair<>(new Integer[] {240, 147, 0}, null));
    lightOptionsMap.put(
        LightCode.LOCKED_ON_NOTE,
        new Pair<>(new Integer[] {240, 147, 0}, new StrobeAnimation(240, 147, 0, 0, .05, 63)));
    lightOptionsMap.put(LightCode.RESTING_RED, new Pair<>(new Integer[] {255, 0, 0}, null));
    lightOptionsMap.put(
        LightCode.LARSON,
        new Pair<>(
            new Integer[] {255, 255, 255},
            new LarsonAnimation(255, 255, 255, 0, .25, 63, LarsonAnimation.BounceMode.Center, 3)));
    lightOptionsMap.put(
        LightCode.TWINKLE,
        new Pair<Integer[], Animation>(white, new FireAnimation(1, .25, 63, 8, 8)));
  }

  public void setLEDColor(LightCode light) {
    Logger.recordOutput("Candle/State", light);
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() {
    if (lightOptionsMap.get(currentLightStatus).getSecond() != null) {
      candle.animate(lightOptionsMap.get(currentLightStatus).getSecond());

      Logger.recordOutput("Candle/Running Anim", true);
    } else {
      var r = lightOptionsMap.get(currentLightStatus).getFirst()[0];
      var g = lightOptionsMap.get(currentLightStatus).getFirst()[1];
      var b = lightOptionsMap.get(currentLightStatus).getFirst()[2];

      Logger.recordOutput("Candle/RGB", r + "," + g + "," + b);
      candle.clearAnimation(0);
      candle.setLEDs(r, g, b);
      Logger.recordOutput("Candle/Running Anim", false);
    }
  }

  public static class Commands {
    public static Command setLEDColor(LightCode light) {
      return new InstantCommand(() -> Robot.candle.setLEDColor(light));
    }
  }
}
