package frc.robot;

/**
 * Robot configuration class containing global settings such as operation mode, debug flags, and
 * control loop timing. All values should be treated as compile-time constants.
 */
public final class Config {
  // Immutable configuration constants
  public static final double LOOP_PERIOD_SEC = 0.02; // Main control loop period in seconds (50Hz)
  public static final boolean IS_LIVE_DEBUG = false; // Enable live debugging outputs
  public static final Mode MODE = Mode.REAL; // Current operating mode

  /** Robot operating mode enumeration */
  public enum Mode {
    REAL, // Physical robot operation
    SIM, // Simulation mode
    REPLAY // Log replay mode
  }

  /** Pre-deployment configuration validation */
  public static final class DeployChecker {
    private static final String DEPLOY_ERROR = "Cannot deploy, invalid robot selected: ";

    /**
     * Validates that the configuration is suitable for physical robot deployment.
     *
     * @throws IllegalStateException if attempting to deploy in non-REAL mode
     */
    public static void validate() {
      if (MODE != Mode.REAL) {
        exitWithError(DEPLOY_ERROR + MODE);
      }
    }
  }

  /** Pre-simulation configuration validation */
  public static final class SimChecker {
    private static final String SIM_ERROR = "Cannot simulate, invalid robot selected: ";

    /**
     * Validates that the configuration is suitable for simulation.
     *
     * @throws IllegalStateException if attempting to simulate in REAL mode
     */
    public static void validate() {
      if (MODE == Mode.REAL) {
        exitWithError(SIM_ERROR + MODE);
      }
    }
  }

  /** Pre-merge configuration validation */
  public static final class MergeChecker {
    private static final String MERGE_ERROR =
        "Do not merge - non-default constants are configured.";

    /**
     * Validates that the configuration uses default values before code merging.
     *
     * @throws IllegalStateException if non-default values are detected
     */
    public static void validate() {
      if (MODE != Mode.REAL || IS_LIVE_DEBUG) {
        exitWithError(MERGE_ERROR);
      }
    }
  }

  /**
   * Terminates the application with an error message.
   *
   * @param message The error message to display
   */
  private static void exitWithError(String message) {
    System.err.println(message);
    System.exit(1);
  }

  // Prevent instantiation
  private Config() {
    throw new UnsupportedOperationException("Utility class cannot be instantiated");
  }
}
