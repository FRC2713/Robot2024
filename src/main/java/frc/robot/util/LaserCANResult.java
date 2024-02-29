package frc.robot.util;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public enum LaserCANResult {
  // https://github.com/GrappleRobotics/libgrapplefrc/blob/master/src/main/java/au/grapplerobotics/LaserCan.java
  VALID_MEASUREMENT(0),
  NOISE_ISSUE(1),
  WEAK_SIGNAL(2),
  OUT_OF_BOUNDS(4),
  WRAPAROUND(7),
  UNKNOWN(-1);

  private final int laserCanIntResult;

  public static LaserCANResult fromInt(int x) {
    for (var result : LaserCANResult.values()) {
      if (x == result.laserCanIntResult) {
        return result;
      }
    }

    return UNKNOWN;
  }
}
