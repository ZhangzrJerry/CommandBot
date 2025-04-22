package frc.robot.utils;

public class Gains {
  public record PidGains(double kP, double kI, double kD) {}

  public record PidfGains(double kP, double kI, double kD, double kS, double kG) {}

  private Gains() {}
}
