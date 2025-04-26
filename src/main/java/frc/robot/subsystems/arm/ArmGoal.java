package frc.robot.subsystems.arm;

import frc.robot.utils.dashboard.TunableNumbers;
import java.util.Map;
import lombok.Getter;

public class ArmGoal extends TunableNumbers {
  public static final String SHOULDER_HEIGHT_METER = "ShoulderHeightMeter";
  public static final String ELBOW_POSITION_DEGREE = "ElbowPositionDegree";

  @Getter private final String name;

  public ArmGoal(String baseKey, double shoulderHeightMeter, double elbowPositionDegree) {
    super(
        baseKey,
        Map.of(
            SHOULDER_HEIGHT_METER, shoulderHeightMeter,
            ELBOW_POSITION_DEGREE, elbowPositionDegree));
    this.name = baseKey.substring(baseKey.lastIndexOf("/") + 1);
  }

  public double getShoulderHeightMeter() {
    return get(SHOULDER_HEIGHT_METER);
  }

  public double getElbowPositionDegree() {
    return get(ELBOW_POSITION_DEGREE);
  }

  public static final ArmGoal START = new ArmGoal("Arm/Goal/Start", 0.0, 107.0);
  public static final ArmGoal IDLE = new ArmGoal("Arm/Goal/Idle", 0.2, -65.0);
  public static final ArmGoal CORAL_L1_SCORE =
      new ArmGoal("Arm/Goal/CoralL1Score", 0.4 - 0.04, -32.0);
  public static final ArmGoal CORAL_L2_SCORE =
      new ArmGoal("Arm/Goal/CoralL2Score", 0.64 - 0.04, -32.0);
  public static final ArmGoal CORAL_L3_SCORE =
      new ArmGoal("Arm/Goal/CoralL3Score", 1.06 - 0.04, -34.0);
  public static final ArmGoal CORAL_L4_SCORE =
      new ArmGoal("Arm/Goal/CoralL4Score", 1.7 - 0.04, -34.0);
  public static final ArmGoal ALGAE_LOW_PICK = new ArmGoal("Arm/Goal/AlgaeLowPick", 0.2, -131.0);
  public static final ArmGoal ALGAE_HIGH_PICK = new ArmGoal("Arm/Goal/AlgaeHighPick", 0.6, -132.0);
  public static final ArmGoal ALGAE_AUTO_LOW_PICK =
      new ArmGoal("Arm/Goal/AlgaeAutoLowPick", 0.5, -90.0);
  public static final ArmGoal ALGAE_AUTO_HIGH_PICK =
      new ArmGoal("Arm/Goal/AlgaeAutoHighPick", 0.7, -90.0);
  public static final ArmGoal ALGAE_PROCESSOR_SCORE =
      new ArmGoal("Arm/Goal/AlgaeProcessorScore", 0.0, -180.0);
  public static final ArmGoal ALGAE_NET_SCORE = new ArmGoal("Arm/Goal/AlgaeNetScore", 1.5, -45.0);
  public static final ArmGoal ALGAE_GROUND_PICK =
      new ArmGoal("Arm/Goal/AlgaeGroundPick", 0.25, 45.0);
  public static final ArmGoal CLIMB = new ArmGoal("Arm/Goal/Climb", 0.65, -180.0);
  public static final ArmGoal HOME = new ArmGoal("Arm/Goal/Home", 0.0, 0.0);
}
