package frc.robot.subsystems.swerveIO.module;

public enum SwerveModuleName {
  FRONT_LEFT("FrontLeft"),
  FRONT_RIGHT("FrontRight"),
  BACK_LEFT("BackLeft"),
  BACK_RIGHT("BackRight");

  private final String string;

  SwerveModuleName(String name) {
    string = name;
  }

  @Override
  public String toString() {
    return string;
  }
}
