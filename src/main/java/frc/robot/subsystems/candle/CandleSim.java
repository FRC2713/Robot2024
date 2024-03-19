package frc.robot.subsystems.candle;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import org.littletonrobotics.junction.Logger;

public class CandleSim extends CANdle {
  public CandleSim() {
    super(0);
  }

  @Override
  public ErrorCode setLEDs(int r, int g, int b) {
    Logger.recordOutput("Candle/RGB", r + ", " + g + ", " + b);
    return ErrorCode.OK;
  }

  @Override
  public ErrorCode animate(Animation animation) {
    return ErrorCode.OK;
  }
}
