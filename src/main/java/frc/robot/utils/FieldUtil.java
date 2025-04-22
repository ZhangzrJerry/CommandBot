package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldUtil {
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
  }
}
