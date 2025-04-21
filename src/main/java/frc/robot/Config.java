package frc.robot;

public final class Config {
  public static final double LOOP_PERIOD_SEC = 0.02;
  public static final boolean IS_LIVE_DEBUG = false;
  public static final Mode MODE = Mode.REAL;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  /** Checks whether the correct mode is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      System.out.println("Deploying code ...");
      if (MODE != Mode.REAL) {
        System.err.println("Cannot deploy, invalid robot selected: " + MODE);
        System.exit(1);
      }
    }
  }

  /** Checks whether the correct mode is selected when simulating. */
  public static class CheckSim {
    public static void main(String... args) {
      System.out.println("Simulating code");
      if (MODE == Mode.REAL) {
        System.err.println("Cannot sim, invalid robot selected: " + MODE);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (MODE != Mode.REAL || IS_LIVE_DEBUG) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
