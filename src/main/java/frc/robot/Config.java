package frc.robot;

/**
 * Robot configuration class containing global settings such as operation mode, debug flags, and
 * control loop timing.
 */
public final class Config {
  // 不可变配置常量
  public static final double LOOP_PERIOD_SEC = 0.02;
  public static final boolean IS_LIVE_DEBUG = false;
  public static final Mode MODE = Mode.REAL;

  /** 运行模式枚举 */
  public enum Mode {
    REAL, // 真实机器人模式
    SIM, // 仿真模式
    REPLAY // 回放模式
  }

  /** 部署前检查配置是否正确 */
  public static final class DeployChecker {
    private static final String DEPLOY_ERROR = "Cannot deploy, invalid robot selected: ";

    public static void validate() {
      if (MODE != Mode.REAL) {
        exitWithError(DEPLOY_ERROR + MODE);
      }
    }
  }

  /** 仿真前检查配置是否正确 */
  public static final class SimChecker {
    private static final String SIM_ERROR = "Cannot sim, invalid robot selected: ";

    public static void validate() {
      if (MODE == Mode.REAL) {
        exitWithError(SIM_ERROR + MODE);
      }
    }
  }

  /** 合并代码前检查配置是否为默认值 */
  public static final class MergeChecker {
    private static final String MERGE_ERROR = "Do not merge, non-default constants are configured.";

    public static void validate() {
      if (MODE != Mode.REAL || IS_LIVE_DEBUG) {
        exitWithError(MERGE_ERROR);
      }
    }
  }

  // 私有工具方法，用于输出错误并退出
  private static void exitWithError(String message) {
    System.err.println(message);
    System.exit(1);
  }

  // 防止实例化
  private Config() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }
}
