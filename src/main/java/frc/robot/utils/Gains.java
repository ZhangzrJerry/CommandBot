package frc.robot.utils;

public class Gains {
  public record PidGains(double kP, double kI, double kD) {}

  public record PdsGains(double kP, double kD, double kS) {}

  public record PidsgGains(double kP, double kI, double kD, double kS, double kG) {}

  private Gains() {}
}
