package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.utils.Gains.GainsImpl;
import frc.robot.utils.Gains.KpdGainsImpl;
import frc.robot.utils.dashboard.TunableGains.TunablePidsgGains;
import frc.robot.utils.dashboard.TunableNumber;

/** Arm configuration class, contains all arm-related configuration parameters */
class ArmConfig {
  // Arm height related configuration
  private static final double SHOULDER_HEIGHT_RATIO = 1.0 / 3.0;
  protected static final double SHOULDER_LEVEL_HEIGHT_OFFSET = 0.04;

  // Arm movement related configuration
  protected static final TunableNumber TRANSITION_ELEVATOR_HEIGHT_METER =
      new TunableNumber("Arm/TransitionElevatorHeightMeter", 0.38);
  protected static final TunableNumber MIN_SAFE_GROUND_INTAKE_DODGE_ELEVATOR_HEIGHT_METER =
      new TunableNumber("Arm/MinSafeGroundIntakeDodgeElevatorHeightMeter", 0.65);

  // Shoulder joint configuration
  protected static final TunableNumber SHOULDER_TOLERANCE_METER =
      new TunableNumber("Arm/Shoulder/ToleranceMeter", 0.1);
  protected static final TunableNumber SHOULDER_STOP_TOLERANCE_METER_PER_SEC =
      new TunableNumber("Arm/Shoulder/StopToleranceMeterPerSec", 0.1);
  protected static final TunableNumber
      SHOULDER_STATIC_CHARACTERIZATION_VELOCITY_THRESH_METER_PER_SEC =
          new TunableNumber("Arm/Shoulder/StaticCharacterizationVelocityThreshMeterPerSec", 0.01);
  protected static final TunableNumber SHOULDER_HOMING_CURRENT_AMP =
      new TunableNumber("Arm/Shoulder/HomingCurrentAmp", -3.0);
  protected static final TunableNumber SHOULDER_HOMING_TIME_SECS =
      new TunableNumber("Arm/Shoulder/HomingTimeSecs", 0.2);
  protected static final TunableNumber SHOULDER_HOMING_VELOCITY_THRESH_METER_PER_SEC =
      new TunableNumber("Arm/Shoulder/HomingVelocityThreshMeterPerSec", 0.05);

  // Elbow joint configuration
  protected static final TunableNumber ELBOW_TOLERANCE_DEGREE =
      new TunableNumber("Arm/Elbow/ToleranceDegree", 12.0);
  protected static final TunableNumber ELBOW_STOP_TOLERANCE_DEGREE_PER_SEC =
      new TunableNumber("Arm/Elbow/StopToleranceDegreePerSec", 10.0);
  protected static final TunableNumber
      ELBOW_STATIC_CHARACTERIZATION_VELOCITY_THRESH_DEGREE_PER_SEC =
          new TunableNumber("Arm/Elbow/StaticCharacterizationVelocityThreshDegreePerSec", 10.0);
  protected static final TunableNumber ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE =
      new TunableNumber("Arm/Elbow/AvoidReefAlgaePositionDegree", -70.0);

  // Arm physical parameters
  static final double ELBOW_CANCODER_OFFSET = 0.13427734375;
  static final double ELBOW_REDUCTION = (68.0 / 10.0) * (72.0 / 16.0);
  static final double SHOULDER_INIT_HEIGHT_METER = 0.0;
  static final double SHOULDER_MAX_HEIGHT_METER = 1.76025;
  static final double SHOULDER_SINGLE_LEVEL_HEIGHT_METER =
      SHOULDER_MAX_HEIGHT_METER * SHOULDER_HEIGHT_RATIO;
  static final double SHOULDER_MIN_HEIGHT_METER = 0.0;
  static final double SHOULDER_METER_PER_ROTATION = 1.76025 / 9.7119140625;
  static final double SHOULDER_REDUCTION = 50.0 / 12.0;

  // PID gain configuration
  protected static final TunablePidsgGains ELBOW_GAINS =
      new TunablePidsgGains(
          "Arm/Elbow/Gains",
          Robot.isReal() ? new GainsImpl(450.0, 0.0, 73.0, 8.0, 0.0) : new KpdGainsImpl(3.0, 0.3));

  protected static final TunablePidsgGains SHOULDER_GAINS =
      new TunablePidsgGains(
          "Arm/Shoulder/Gains",
          Robot.isReal() ? new GainsImpl(225, 0.0, 30.0, 24.0, 38) : new KpdGainsImpl(3.0, 0.5));

  // Arm zero position transformation
  protected static final Transform3d L2_ZEROED_TF =
      new Transform3d(0., 0, 0, new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
  protected static final Transform3d L3_ZEROED_TF = new Transform3d();
  protected static final Transform3d L4_ZEROED_TF = new Transform3d();
  protected static final Transform3d ARM_ZEROED_TF =
      new Transform3d(
          0,
          0.5,
          0.,
          new Rotation3d(new Quaternion(0.990691, -0.136134, -3.02277e-17, -3.02277e-17))
              .plus(new Rotation3d(Units.degreesToRadians(-75), 0, 0)));

  /** Get CANcoder configuration */
  protected static CANcoderConfiguration getCancoderConfig(double magnetOffset) {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;
    return config;
  }

  /** Get elbow motor configuration */
  static TalonFXConfiguration getElbowTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0 =
        new Slot0Configs()
            .withKP(ELBOW_GAINS.getKP())
            .withKI(ELBOW_GAINS.getKI())
            .withKD(ELBOW_GAINS.getKD())
            .withKS(ELBOW_GAINS.getKS())
            .withKG(ELBOW_GAINS.getKG())
            .withGravityType(GravityTypeValue.Arm_Cosine);

    config.TorqueCurrent.PeakForwardTorqueCurrent = 170.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -170.0;

    config.CurrentLimits.SupplyCurrentLimit = 37.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 200.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.RotorToSensorRatio = ELBOW_REDUCTION;
    config.ClosedLoopGeneral.ContinuousWrap = false;

    return config;
  }

  /** Get shoulder motor configuration */
  static TalonFXConfiguration getShoulderTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0 =
        new Slot0Configs()
            .withKP(SHOULDER_GAINS.getKP())
            .withKI(SHOULDER_GAINS.getKI())
            .withKD(SHOULDER_GAINS.getKD())
            .withKS(SHOULDER_GAINS.getKS())
            .withKG(SHOULDER_GAINS.getKG())
            .withGravityType(GravityTypeValue.Elevator_Static);

    config.CurrentLimits.SupplyCurrentLimit = 50.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = SHOULDER_REDUCTION;

    return config;
  }
}
