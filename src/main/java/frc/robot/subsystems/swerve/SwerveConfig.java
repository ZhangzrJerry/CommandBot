package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Gains.GainsImpl;
import frc.robot.utils.Gains.KpGainsImpl;
import frc.robot.utils.dashboard.TunableGains.TunablePidsgGains;
import frc.robot.utils.dashboard.TunableNumber;

public class SwerveConfig {
  protected static final TunableNumber maxTiltAccelXMeterPerSecPerLoop =
      new TunableNumber("Swerve/MaxTiltAccelXMeterPerSecPerLoop", 80.0);
  protected static final TunableNumber maxTiltAccelYMeterPerSecPerLoop =
      new TunableNumber("Swerve/MaxTiltAccelYMeterPerSecPerLoop", 80.0);
  protected static final TunableNumber maxSkidAccelMeterPerSecPerLoop =
      new TunableNumber(
          "Swerve/MaxSkidAccelMeterPerSecPerLoop",
          SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC * 2.0 / Constants.LOOP_PERIOD_SEC);

  protected static final TunableNumber currentRampFactor =
      new TunableNumber("KsCharacterization/CurrentRampPerSec", 1.0);
  protected static final TunableNumber minVelocity =
      new TunableNumber("KsCharacterization/MinStaticVelocity", 0.1);
  protected static final TunableNumber characterizationSpeedDegreePerSec =
      new TunableNumber("Swerve/WheelRadiusCharacterization/SpeedDegreePerSec", 360.0);

  static final String FL_MODULE_NAME = "FL";
  static final String BL_MODULE_NAME = "BL";
  static final String BR_MODULE_NAME = "BR";
  static final String FR_MODULE_NAME = "FR";

  static final double ODOMETRY_FREQUENCY_HZ = 250.0;

  static final boolean ENABLE_TRAJECTORY_FF = true;

  protected static final double WHEELBASE_LENGTH_METER = 0.2967 * 2.0;
  protected static final double WHEELBASE_WIDTH_METER = 0.2967 * 2.0;
  protected static final double WHEELBASE_DIAGONAL_METER =
      Math.hypot(WHEELBASE_LENGTH_METER, WHEELBASE_WIDTH_METER);

  public static final double MAX_TRANSLATION_VEL_METER_PER_SEC = 4.925568; // WCP X3T10
  public static final double MAX_TRANSLATION_ACC_METERS_PER_SEC = 80;
  public static final double MAX_ANGULAR_VEL_RAD_PER_SEC =
      MAX_TRANSLATION_VEL_METER_PER_SEC / (WHEELBASE_DIAGONAL_METER / 2.0);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEELBASE_LENGTH_METER / 2.0, WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0));

  static final double WHEEL_RADIUS_METER = 0.0479;
  static final double FL_CANCODER_OFFSET = 0.30859375;
  static final double BL_CANCODER_OFFSET = -0.367919921875;
  static final double BR_CANCODER_OFFSET = 0.48046875;
  static final double FR_CANCODER_OFFSET = 0.070068359375;
  static final CANcoderConfiguration FL_CANCODER_CONFIG = getCancoderConfig(FL_CANCODER_OFFSET);
  static final CANcoderConfiguration BL_CANCODER_CONFIG = getCancoderConfig(BL_CANCODER_OFFSET);
  static final CANcoderConfiguration BR_CANCODER_CONFIG = getCancoderConfig(BR_CANCODER_OFFSET);
  static final CANcoderConfiguration FR_CANCODER_CONFIG = getCancoderConfig(FR_CANCODER_OFFSET);

  /// WCP X2 Reductions
  /// https://docs.wcproducts.com/wcp-swerve-x2/general-info/ratio-options
  /// - X1T10: (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0) = 7.67
  /// - X1T11: (54.0 / 11.0) * (18.0 / 38.0) * (45.0 / 15.0) = 6.98
  /// - X1T12: (54.0 / 12.0) * (18.0 / 38.0) * (45.0 / 15.0) = 6.39
  /// - X2T10: (54.0 / 10.0) * (16.0 / 38.0) * (45.0 / 15.0) = 6.82
  /// - X2T11: (54.0 / 11.0) * (16.0 / 38.0) * (45.0 / 15.0) = 6.20
  /// - X2T12: (54.0 / 12.0) * (16.0 / 38.0) * (45.0 / 15.0) = 5.68
  /// - X3T10: (54.0 / 10.0) * (16.0 / 40.0) * (45.0 / 15.0) = 6.48
  /// - X3T11: (54.0 / 11.0) * (16.0 / 40.0) * (45.0 / 15.0) = 5.89
  /// - X3T12: (54.0 / 12.0) * (16.0 / 40.0) * (45.0 / 15.0) = 5.40
  /// - X4T10: (54.0 / 10.0) * (14.0 / 40.0) * (45.0 / 15.0) = 5.67
  /// - X4T11: (54.0 / 11.0) * (14.0 / 40.0) * (45.0 / 15.0) = 5.15
  /// - X4T12: (54.0 / 12.0) * (14.0 / 40.0) * (45.0 / 15.0) = 4.73
  /// - TURN: (88.0 / 16.0) * (22.0 / 27.0) * (27.0 / 10.0) = 12.1
  static final double DRIVE_REDUCTION = (54.0 / 10.0) * (16.0 / 40.0) * (45.0 / 15.0);
  static final double STEER_REDUCTION = (88.0 / 16.0) * (22.0 / 27.0) * (27.0 / 10.0);

  static final double DRIVE_FF_KT =
      DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_REDUCTION).KtNMPerAmp;

  static TunablePidsgGains DRIVE_GAINS =
      new TunablePidsgGains(
          "Swerve/Module/DriveGains",
          Robot.isReal() ? new GainsImpl(7.0, 0.0, 0.0, 0.0, 0.0) : new KpGainsImpl(0.3));
  static TunablePidsgGains STEER_GAINS =
      new TunablePidsgGains(
          "Swerve/Module/SteerGains",
          Robot.isReal() ? new GainsImpl(2000.0, 0.0, 60.0, 0.0, 0.0) : new KpGainsImpl(10.0));

  static TalonFXConfiguration getX2DriveTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0 =
        new Slot0Configs()
            .withKP(DRIVE_GAINS.getKP())
            .withKI(DRIVE_GAINS.getKI())
            .withKD(DRIVE_GAINS.getKD())
            .withKS(DRIVE_GAINS.getKS())
            .withKG(DRIVE_GAINS.getKG());
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 200.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION;

    return config;
  }

  static TalonFXConfiguration getX2SteerTalonNoEncoderConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0 =
        new Slot0Configs()
            .withKP(STEER_GAINS.getKP())
            .withKI(STEER_GAINS.getKI())
            .withKD(STEER_GAINS.getKD())
            .withKS(STEER_GAINS.getKS())
            .withKG(STEER_GAINS.getKG());
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.RotorToSensorRatio = STEER_REDUCTION;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    return config;
  }

  private static CANcoderConfiguration getCancoderConfig(double magnetOffset) {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;

    return config;
  }

  private static final double WHEELBASE_HEIGHT_METER = .07;
  private static final Rotation3d WHEELBASE_ROTATION3D =
      new Rotation3d(new Quaternion(.5, .5, .5, .5));
  public static final Transform3d FL_ZEROED_TF =
      new Transform3d(
          -WHEELBASE_WIDTH_METER / 2.0,
          -WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
  public static final Transform3d BL_ZEROED_TF =
      new Transform3d(
          -WHEELBASE_WIDTH_METER / 2.0,
          WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
  public static final Transform3d BR_ZEROED_TF =
      new Transform3d(
          WHEELBASE_WIDTH_METER / 2.0,
          WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);
  public static final Transform3d FR_ZEROED_TF =
      new Transform3d(
          WHEELBASE_WIDTH_METER / 2.0,
          -WHEELBASE_LENGTH_METER / 2.0,
          WHEELBASE_HEIGHT_METER,
          WHEELBASE_ROTATION3D);

  public static final double STATIC_CHARACTERIZATION_VELOCITY_THRESH_METER_PER_SEC = 0.01;
}
