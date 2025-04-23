package frc.robot.hardware.communication.phoenix;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixConfigurator {
  public static void configure(String configDescription, Supplier<StatusCode> configFunction) {
    configure(configDescription, configFunction, 5);
  }

  public static void configure(
      String configDescription, Supplier<StatusCode> configFunction, int numTries) {
    StatusCode code = configFunction.get();

    var tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning(
          configDescription + " -> Retrying phoenix 6 device config " + code.getName(), false);
      code = configFunction.get();
      tries++;
    }

    if (code != StatusCode.OK) {
      DriverStation.reportError(
          configDescription
              + " -> Failed to config phoenix 6 device after "
              + numTries
              + " attempts",
          false);
    }
  }

  private PhoenixConfigurator() {} // Prevent instantiation
}
