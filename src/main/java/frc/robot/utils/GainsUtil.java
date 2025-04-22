package frc.robot.utils;

/** Utility class for defining gain configurations */
public class GainsUtil {
  /** Sealed interface for all gain types */
  public sealed interface Gains permits PidGains, PdsGains, PidsgGains {
    String[] getKeys();

    double[] getValues();
  }

  /** PID Gains record */
  public record PidGains(double kP, double kI, double kD) implements Gains {
    @Override
    public String[] getKeys() {
      return new String[] {"kP", "kI", "kD"};
    }

    @Override
    public double[] getValues() {
      return new double[] {kP, kI, kD};
    }
  }

  /** PDS Gains record */
  public record PdsGains(double kP, double kD, double kS) implements Gains {
    @Override
    public String[] getKeys() {
      return new String[] {"kP", "kD", "kS"};
    }

    @Override
    public double[] getValues() {
      return new double[] {kP, kD, kS};
    }
  }

  /** PIDSG Gains record */
  public record PidsgGains(double kP, double kI, double kD, double kS, double kG) implements Gains {
    @Override
    public String[] getKeys() {
      return new String[] {"kP", "kI", "kD", "kS", "kG"};
    }

    @Override
    public double[] getValues() {
      return new double[] {kP, kI, kD, kS, kG};
    }
  }
}
