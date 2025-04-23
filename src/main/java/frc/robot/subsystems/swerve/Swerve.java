package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Ports;
import frc.robot.drivers.dcmotor.DCMotorIO;
import frc.robot.drivers.dcmotor.DCMotorIOKraken;
import frc.robot.drivers.dcmotor.DCMotorIOKrakenCancoder;
import frc.robot.drivers.dcmotor.DCMotorIOSim;
import frc.robot.drivers.gyro.GyroIO;
import frc.robot.drivers.gyro.GyroIOInputsAutoLogged;
import frc.robot.drivers.gyro.GyroIOPigeon2;
import frc.robot.utils.AlertUtil;
import frc.robot.utils.EqualsUtil;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.UnitConverter;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class, EqualsUtil.GeomExtensions.class})
public class Swerve extends SubsystemBase {
  private final LoggedTunableNumber maxTiltAccelXMeterPerSecPerLoop =
      new LoggedTunableNumber("Swerve/MaxTiltAccelXMeterPerSecPerLoop", 80.0);
  private final LoggedTunableNumber maxTiltAccelYMeterPerSecPerLoop =
      new LoggedTunableNumber("Swerve/MaxTiltAccelYMeterPerSecPerLoop", 80.0);
  private final LoggedTunableNumber maxSkidAccelMeterPerSecPerLoop =
      new LoggedTunableNumber(
          "Swerve/MaxSkidAccelMeterPerSecPerLoop",
          SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC * 2.0 / Config.LOOP_PERIOD_SEC);

  private final SwerveModule[] modules = new SwerveModule[4];

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final AlertUtil gyroOfflineAlert =
      new AlertUtil("Gyro offline!", AlertUtil.AlertType.WARNING);

  @Setter private SwerveController controller = new SwerveController() {};
  @Setter private Supplier<Double> customMaxTiltAccelScale = () -> 1.0;

  private SwerveModuleState[] lastGoalModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
      };

  @AutoLogOutput(key = "Swerve/WheeledPose")
  @Getter
  private Pose2d wheeledPose = new Pose2d();

  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];
  private Rotation2d lastGyroYaw = new Rotation2d();

  public static Swerve createSim() {
    DCMotorIO flDriveIO,
        flSteerIO,
        frDriveIO,
        frSteerIO,
        blDriveIO,
        blSteerIO,
        brDriveIO,
        brSteerIO;

    UnitConverter driveRatioConverter =
        UnitConverter.scale(SwerveConfig.WHEEL_RADIUS_METER).withUnits("rad", "m");
    UnitConverter steerRatioConverter = UnitConverter.identity().withUnits("rad", "rad");

    flDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    flSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);
    frDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    frSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);
    blDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    blSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);
    brDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    brSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);

    GyroIO gyroIO = new GyroIO() {};

    return new Swerve(
        flDriveIO, flSteerIO, frDriveIO, frSteerIO, blDriveIO, blSteerIO, brDriveIO, brSteerIO,
        gyroIO);
  }

  public static Swerve createReal() {
    DCMotorIO flDriveIO,
        flSteerIO,
        frDriveIO,
        frSteerIO,
        blDriveIO,
        blSteerIO,
        brDriveIO,
        brSteerIO;

    UnitConverter driveRatioConverter =
        UnitConverter.scale(2 * Math.PI * SwerveConfig.WHEEL_RADIUS_METER).withUnits("rot", "m");
    UnitConverter steerRatioConverter = UnitConverter.scale(2 * Math.PI).withUnits("rot", "rad");

    flDriveIO =
        new DCMotorIOKraken(
            "flDrive",
            Ports.Can.FL_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    flSteerIO =
        new DCMotorIOKrakenCancoder(
            "flSteer",
            Ports.Can.FL_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.FL_STEER_SENSOR,
            SwerveConfig.FL_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.FL_CANCODER_OFFSET).withUnits("rot", "rot"));

    frDriveIO =
        new DCMotorIOKraken(
            "frDrive",
            Ports.Can.FR_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    frSteerIO =
        new DCMotorIOKrakenCancoder(
            "frSteer",
            Ports.Can.FR_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.FR_STEER_SENSOR,
            SwerveConfig.FR_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.FR_CANCODER_OFFSET).withUnits("rot", "rot"));

    blDriveIO =
        new DCMotorIOKraken(
            "blDrive",
            Ports.Can.BL_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    blSteerIO =
        new DCMotorIOKrakenCancoder(
            "blSteer",
            Ports.Can.BL_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.BL_STEER_SENSOR,
            SwerveConfig.BL_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.BL_CANCODER_OFFSET).withUnits("rot", "rot"));

    brDriveIO =
        new DCMotorIOKraken(
            "brDrive",
            Ports.Can.BR_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    brSteerIO =
        new DCMotorIOKrakenCancoder(
            "brSteer",
            Ports.Can.BR_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.BR_STEER_SENSOR,
            SwerveConfig.BR_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.BR_CANCODER_OFFSET).withUnits("rot", "rot"));

    GyroIO gyroIO = new GyroIOPigeon2(Ports.Can.CHASSIS_PIGEON);

    return new Swerve(
        flDriveIO, flSteerIO, frDriveIO, frSteerIO, blDriveIO, blSteerIO, brDriveIO, brSteerIO,
        gyroIO);
  }

  public static Swerve createIO() {
    return new Swerve(
        new DCMotorIO() {},
        new DCMotorIO() {},
        new DCMotorIO() {},
        new DCMotorIO() {},
        new DCMotorIO() {},
        new DCMotorIO() {},
        new DCMotorIO() {},
        new DCMotorIO() {},
        new GyroIO() {});
  }

  private Swerve(
      DCMotorIO flDriveIO,
      DCMotorIO flSteerIO,
      DCMotorIO frDriveIO,
      DCMotorIO frSteerIO,
      DCMotorIO blDriveIO,
      DCMotorIO blSteerIO,
      DCMotorIO brDriveIO,
      DCMotorIO brSteerIO,
      GyroIO gyroIO) {
    this.gyroIO = gyroIO;

    modules[0] = new SwerveModule(flDriveIO, flSteerIO, "ModuleFL");
    modules[1] = new SwerveModule(blDriveIO, blSteerIO, "ModuleBL");
    modules[2] = new SwerveModule(brDriveIO, brSteerIO, "ModuleBR");
    modules[3] = new SwerveModule(frDriveIO, frSteerIO, "ModuleFR");

    for (int i = 0; i < modules.length; i++) {
      lastModulePositions[i] = modules[i].getPosition();
    }
    lastGyroYaw = gyroIO.getYaw();
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    gyroOfflineAlert.set(!gyroInputs.connected);

    for (SwerveModule module : modules) {
      module.updateInputs();
    }

    // Update wheeled pose
    var modulePositions = getPositions();
    var twist = SwerveConfig.SWERVE_KINEMATICS.toTwist2d(lastModulePositions, modulePositions);
    lastModulePositions = modulePositions.clone();

    if (lastGyroYaw != null) {
      var gyroYaw = gyroIO.getYaw();
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
      lastGyroYaw = gyroYaw;
    }

    wheeledPose = wheeledPose.exp(twist);

    ChassisSpeeds currentVel = getCurrentVel();
    ChassisSpeeds goalVel = controller.getChassisSpeeds();

    if (EqualsUtil.epsilonEquals(goalVel.vxMetersPerSecond, 0.0)
        && EqualsUtil.epsilonEquals(goalVel.vyMetersPerSecond, 0.0)
        && EqualsUtil.epsilonEquals(goalVel.omegaRadiansPerSecond, 0.0)) {
      for (int i = 0; i < modules.length; ++i) {
        modules[i].stop();
      }
    }

    var rawGoalModuleStates = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(goalVel);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        rawGoalModuleStates, SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC);
    goalVel = SwerveConfig.SWERVE_KINEMATICS.toChassisSpeeds(rawGoalModuleStates);

    // 1690 Orbit accel limitation
    goalVel = applyAccelLimitation(currentVel, goalVel);

    // Dynamics compensation
    goalVel = ChassisSpeeds.discretize(goalVel, Config.LOOP_PERIOD_SEC);

    // Use last goal angle for module if chassis want stop completely
    var goalModuleStates = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(goalVel);
    if (goalVel.toTwist2d().epsilonEquals(new Twist2d())) {
      for (int i = 0; i < modules.length; i++) {
        goalModuleStates[i].angle = lastGoalModuleStates[i].angle;
        goalModuleStates[i].speedMetersPerSecond = 0.0;
      }
    }

    var optimizedGoalModuleStates = new SwerveModuleState[4];
    var optimizedGoalModuleTorques = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      // Optimize setpoints
      optimizedGoalModuleStates[i] = goalModuleStates[i];
      optimizedGoalModuleStates[i].optimize(modules[i].getState().angle);

      optimizedGoalModuleTorques[i] =
          new SwerveModuleState(0.0, optimizedGoalModuleStates[i].angle);
      modules[i].setState(optimizedGoalModuleStates[i]);
    }

    lastGoalModuleStates = goalModuleStates;

    Logger.recordOutput("Swerve/SwerveStates/GoalModuleStates", goalModuleStates);
    Logger.recordOutput("Swerve/FinalGoalVel", goalVel);
    Logger.recordOutput("Swerve/SwerveStates/OptimizedGoalModuleStates", optimizedGoalModuleStates);
    Logger.recordOutput(
        "Swerve/SwerveStates/OptimizedGoalModuleTorques", optimizedGoalModuleTorques);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  @AutoLogOutput(key = "Swerve/ModuleStates")
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getCurrentVel() {
    return SwerveConfig.SWERVE_KINEMATICS.toChassisSpeeds(getStates());
  }

  private ChassisSpeeds applyAccelLimitation(
      final ChassisSpeeds currentVel, final ChassisSpeeds goalVel) {
    var currentTranslationVel =
        new Translation2d(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond);

    var goalTranslationVel =
        new Translation2d(goalVel.vxMetersPerSecond, goalVel.vyMetersPerSecond);

    var rawAccelPerLoop =
        goalTranslationVel.minus(currentTranslationVel).div(Config.LOOP_PERIOD_SEC);

    var customMaxTiltAccelScaleVal = customMaxTiltAccelScale.get();
    Logger.recordOutput("Swerve/customMaxTiltAccelScale", customMaxTiltAccelScaleVal);
    var maxTiltAccelXPerLoop = maxTiltAccelXMeterPerSecPerLoop.get() * customMaxTiltAccelScaleVal;
    var maxTiltAccelYPerLoop = maxTiltAccelYMeterPerSecPerLoop.get() * customMaxTiltAccelScaleVal;

    var tiltLimitedAccelPerLoop =
        new Translation2d(
            MathUtil.clamp(rawAccelPerLoop.getX(), -maxTiltAccelXPerLoop, maxTiltAccelXPerLoop),
            MathUtil.clamp(rawAccelPerLoop.getY(), -maxTiltAccelYPerLoop, maxTiltAccelYPerLoop));

    var skidLimitedAccelPerLoop = new Translation2d();

    if (!EqualsUtil.epsilonEquals(tiltLimitedAccelPerLoop.getNorm(), 0.0)) {
      skidLimitedAccelPerLoop =
          new Translation2d(
              MathUtil.clamp(
                  tiltLimitedAccelPerLoop.getNorm(),
                  -maxSkidAccelMeterPerSecPerLoop.get(),
                  maxSkidAccelMeterPerSecPerLoop.get()),
              tiltLimitedAccelPerLoop.toRotation2d());
    }

    var calculatedDeltaVel = skidLimitedAccelPerLoop.times(Config.LOOP_PERIOD_SEC);

    var limitedGoalVelTranslation = currentTranslationVel.plus(calculatedDeltaVel);

    return new ChassisSpeeds(
        limitedGoalVelTranslation.getX(),
        limitedGoalVelTranslation.getY(),
        goalVel.omegaRadiansPerSecond);
  }
}
