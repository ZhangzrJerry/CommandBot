package frc.robot.subsystems.arm;

import frc.robot.utils.dashboard.TunableNumbers;
import java.util.Map;
import lombok.Getter;

/** Arm target position class, defines various target positions for the arm */
public class ArmGoal extends TunableNumbers {
    // Configuration key names
    public static final String SHOULDER_HEIGHT_METER = "ShoulderHeightMeter";
    public static final String ELBOW_POSITION_DEGREE = "ElbowPositionDegree";

    // Coral related positions
    private static final double CORAL_L1_HEIGHT = 0.4;
    private static final double CORAL_L2_HEIGHT = 0.64;
    private static final double CORAL_L3_HEIGHT = 1.06;
    private static final double CORAL_L4_HEIGHT = 1.7;
    private static final double CORAL_ELBOW_ANGLE = -32.0;
    private static final double CORAL_STATION_COLLECT_ELBOW_ANGLE = -232.0;

    // Algae related positions
    private static final double ALGAE_LOW_PICK_HEIGHT = 0.2;
    private static final double ALGAE_HIGH_PICK_HEIGHT = 0.6;
    private static final double ALGAE_AUTO_PICK_HEIGHT_LOW = 0.5;
    private static final double ALGAE_AUTO_PICK_HEIGHT_HIGH = 0.7;
    private static final double ALGAE_LOW_PICK_ELBOW_ANGLE = -131.0;
    private static final double ALGAE_HIGH_PICK_ELBOW_ANGLE = -132.0;
    private static final double ALGAE_AUTO_PICK_ELBOW_ANGLE = -90.0;
    private static final double ALGAE_GROUND_PICK_ELBOW_ANGLE = 45.0;

    @Getter
    private final String name;

    /**
     * Constructor
     *
     * @param baseKey             Base key name
     * @param shoulderHeightMeter Shoulder height (meters)
     * @param elbowPositionDegree Elbow angle (degrees)
     */
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

    // Basic positions
    public static final ArmGoal START = new ArmGoal("Arm/Goal/Start", 0.0, 107.0);
    public static final ArmGoal IDLE = new ArmGoal("Arm/Goal/Idle", 0.2, -65.0);
    public static final ArmGoal HOME = new ArmGoal("Arm/Goal/Home", 0.0, 0.0);
    public static final ArmGoal CLIMB = new ArmGoal("Arm/Goal/Climb", 0.7, -180.0);

    // Coral scoring positions
    public static final ArmGoal CORAL_L1_SCORE = new ArmGoal(
            "Arm/Goal/CoralL1Score",
            CORAL_L1_HEIGHT - ArmConfig.SHOULDER_LEVEL_HEIGHT_OFFSET,
            CORAL_ELBOW_ANGLE);
    public static final ArmGoal CORAL_L2_SCORE = new ArmGoal(
            "Arm/Goal/CoralL2Score",
            CORAL_L2_HEIGHT - ArmConfig.SHOULDER_LEVEL_HEIGHT_OFFSET,
            CORAL_ELBOW_ANGLE);
    public static final ArmGoal CORAL_L3_SCORE = new ArmGoal(
            "Arm/Goal/CoralL3Score",
            CORAL_L3_HEIGHT - ArmConfig.SHOULDER_LEVEL_HEIGHT_OFFSET,
            CORAL_ELBOW_ANGLE);
    public static final ArmGoal CORAL_L4_SCORE = new ArmGoal(
            "Arm/Goal/CoralL4Score",
            CORAL_L4_HEIGHT - ArmConfig.SHOULDER_LEVEL_HEIGHT_OFFSET,
            CORAL_ELBOW_ANGLE);
    public static final ArmGoal CORAL_STATION_COLLECT = new ArmGoal("Arm/Goal/CoralStationCollect", 0.05,
            CORAL_STATION_COLLECT_ELBOW_ANGLE);

    // Algae related positions
    public static final ArmGoal ALGAE_LOW_PICK = new ArmGoal("Arm/Goal/AlgaeLowPick", ALGAE_LOW_PICK_HEIGHT,
            ALGAE_LOW_PICK_ELBOW_ANGLE);
    public static final ArmGoal ALGAE_HIGH_PICK = new ArmGoal("Arm/Goal/AlgaeHighPick", ALGAE_HIGH_PICK_HEIGHT,
            ALGAE_HIGH_PICK_ELBOW_ANGLE);
    public static final ArmGoal ALGAE_AUTO_LOW_PICK = new ArmGoal(
            "Arm/Goal/AlgaeAutoLowPick", ALGAE_AUTO_PICK_HEIGHT_LOW, ALGAE_AUTO_PICK_ELBOW_ANGLE);
    public static final ArmGoal ALGAE_AUTO_HIGH_PICK = new ArmGoal(
            "Arm/Goal/AlgaeAutoHighPick", ALGAE_AUTO_PICK_HEIGHT_HIGH, ALGAE_AUTO_PICK_ELBOW_ANGLE);
    public static final ArmGoal ALGAE_PROCESSOR_SCORE = new ArmGoal("Arm/Goal/AlgaeProcessorScore", 0.0, -180.0);
    public static final ArmGoal ALGAE_NET_SCORE = new ArmGoal("Arm/Goal/AlgaeNetScore", 1.5, -45.0);
    public static final ArmGoal ALGAE_GROUND_PICK = new ArmGoal("Arm/Goal/AlgaeGroundPick", 0.25,
            ALGAE_GROUND_PICK_ELBOW_ANGLE);
}
