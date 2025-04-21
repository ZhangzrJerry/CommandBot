package frc.robot.virtuals.visualize;

/**
 * Configuration class for visualization component IDs. These IDs should match the part order in
 * config.json in AdvantageScope.
 */
public final class VisualizeConfig {
  private VisualizeConfig() {}

  public static final class Ids {
    private Ids() {}

    public static final int ROBOT_FRAME = -1;
    public static final int SWERVE_FL = 0;
    public static final int SWERVE_BL = 1;
    public static final int SWERVE_BR = 2;
    public static final int SWERVE_FR = 3;
    public static final int ELEVATOR_L2 = 4;
    public static final int ELEVATOR_L3 = 5;
    public static final int ELEVATOR_CARRIAGE = 6;
    public static final int ARM = 7;
    public static final int INTAKE = 8;
    public static final int CLIMBER = 9;
    public static final int ALGAE = 10;
    public static final int CORAL = 11;
  }
}
