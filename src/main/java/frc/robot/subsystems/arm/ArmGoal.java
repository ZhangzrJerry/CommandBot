package frc.robot.subsystems.arm;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.dashboard.TunableNumbers;

public class ArmGoal extends TunableNumbers {
  public static final String SHOULDER_HEIGHT_METER = "ShoulderHeightMeter";
  public static final String ELBOW_POSITION_DEGREE = "ElbowPositionDegree";

  public ArmGoal(String baseKey, double shoulderHeightMeter, double elbowPositionDegree) {
    super(baseKey,
        Map.of(
            SHOULDER_HEIGHT_METER, shoulderHeightMeter,
            ELBOW_POSITION_DEGREE, elbowPositionDegree));
  }

  public double getShoulderHeightMeter() {
    return get(SHOULDER_HEIGHT_METER);
  }

  public double getElbowPositionRad() {
    return Units.degreesToRadians(get(ELBOW_POSITION_DEGREE));
  }

  public static final ArmGoal START = new ArmGoal("Arm/Goal/Start", 0.4, -230.0);
  public static final ArmGoal IDLE = new ArmGoal("Arm/Goal/Idle", 0.0, -90.0);
  public static final ArmGoal DEFENCE = new ArmGoal("Arm/Goal/Defence", 0.3, -250.0);
  public static final ArmGoal CORAL_L1_SCORE = new ArmGoal("Arm/Goal/CoralL1Score", 0.04, -190.0);
  public static final ArmGoal CORAL_L2_SCORE = new ArmGoal("Arm/Goal/CoralL2Score", 0.27, -200.0);
  public static final ArmGoal CORAL_L3_SCORE = new ArmGoal("Arm/Goal/CoralL3Score", 0.67, -200.0);
  public static final ArmGoal CORAL_L4_SCORE = new ArmGoal("Arm/Goal/CoralL4Score", 1.55, -210.0);
  public static final ArmGoal ALGAE_LOW_PICK = new ArmGoal("Arm/Goal/AlgaeLowPick", 0.2, -131.0);
  public static final ArmGoal ALGAE_HIGH_PICK = new ArmGoal("Arm/Goal/AlgaeHighPick", 0.6, -132.0);
  public static final ArmGoal ALGAE_AUTO_LOW_PICK = new ArmGoal("Arm/Goal/AlgaeAutoLowPick", 0.2, -131.0);
  public static final ArmGoal ALGAE_AUTO_HIGH_PICK = new ArmGoal("Arm/Goal/AlgaeAutoHighPick", 0.6, -132.0);
  public static final ArmGoal ALGAE_PROCESSOR_SCORE = new ArmGoal("Arm/Goal/AlgaeProcessorScore", 0.0, -180.0);
  public static final ArmGoal ALGAE_NET_SCORE = new ArmGoal("Arm/Goal/AlgaeNetScore", 1.5, -45.0);
  public static final ArmGoal CLIMB = new ArmGoal("Arm/Goal/Climb", 0.4, -150.0);
  public static final ArmGoal HOME = new ArmGoal("Arm/Goal/Home", 0.0, -90.0);
}