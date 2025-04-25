package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfx;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfxCancoder;
import frc.robot.interfaces.hardwares.sensors.gyro.GyroIO;
import frc.robot.interfaces.hardwares.sensors.gyro.GyroIOPigeon2;
import frc.robot.interfaces.threads.wheeled.WheeledOdometryPhoenixThread;
import frc.robot.interfaces.threads.wheeled.WheeledOdometrySimThread;
import frc.robot.interfaces.threads.wheeled.WheeledOdometryThread;
import frc.robot.services.VisualizeService;
import frc.robot.utils.math.EqualsUtil;
import frc.robot.utils.math.GeomUtil;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import frc.robot.utils.math.UnitConverter;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class, EqualsUtil.GeomExtensions.class})
public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4];
  @Getter private final SwerveOdometry odometry;
  @Setter private SwerveController controller = new SwerveController() {};

  @Setter private DoubleSupplier customMaxTiltAccelScale = () -> 1.0;
  private SwerveModuleState[] lastGoalModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  public Command registerControllerCmd(SwerveController controller) {
    return Commands.run(
            () -> {
              this.controller = controller;
            },
            this)
        .withName("> Swerve/Register Controller: " + controller.getName());
  }

  public Command resetWheeledPoseCmd(UncertainPose2d pose) {
    return Commands.runOnce(
            () -> {
              odometry.resetWheeledPose(pose);
            })
        .withName("> Swerve/Reset Wheeled Pose");
  }

  public Command resetGyroHeadingCmd(Rotation2d yaw) {
    return Commands.runOnce(
            () -> {
              odometry.resetGyroHeading(yaw);
            })
        .withName("> Swerve/Reset Gyro Heading");
  }

  public Pose2d getPose() {
    return odometry.getPose().getPose();
  }

  public UncertainPose2d getUncertainPose2d() {
    return odometry.getPose();
  }

  @Override
  public void periodic() {

    // signalLock.lock();
    // try {
    for (SwerveModule module : modules) {
      module.updateInputs();
    }
    odometry.updateInputs();
    // } finally {
    // signalLock.unlock();
    // }

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
    goalVel = ChassisSpeeds.discretize(goalVel, Constants.LOOP_PERIOD_SEC);

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
        goalTranslationVel.minus(currentTranslationVel).div(Constants.LOOP_PERIOD_SEC);

    var customMaxTiltAccelScaleVal = customMaxTiltAccelScale.getAsDouble();
    Logger.recordOutput("Swerve/customMaxTiltAccelScale", customMaxTiltAccelScaleVal);
    var maxTiltAccelXPerLoop =
        SwerveConfig.maxTiltAccelXMeterPerSecPerLoop.get() * customMaxTiltAccelScaleVal;
    var maxTiltAccelYPerLoop =
        SwerveConfig.maxTiltAccelYMeterPerSecPerLoop.get() * customMaxTiltAccelScaleVal;

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
                  -SwerveConfig.maxSkidAccelMeterPerSecPerLoop.get(),
                  SwerveConfig.maxSkidAccelMeterPerSecPerLoop.get()),
              tiltLimitedAccelPerLoop.toRotation2d());
    }

    var calculatedDeltaVel = skidLimitedAccelPerLoop.times(Constants.LOOP_PERIOD_SEC);

    var limitedGoalVelTranslation = currentTranslationVel.plus(calculatedDeltaVel);

    return new ChassisSpeeds(
        limitedGoalVelTranslation.getX(),
        limitedGoalVelTranslation.getY(),
        goalVel.omegaRadiansPerSecond);
  }

  public static Swerve createSim() {
    UnitConverter driveRatioConverter =
        UnitConverter.scale(SwerveConfig.WHEEL_RADIUS_METER).withUnits("rad", "m");
    UnitConverter steerRatioConverter = UnitConverter.identity().withUnits("rad", "rad");

    DCMotorIO flDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.SIM_DRIVE_GAINS);
    DCMotorIO flSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.SIM_STEER_GAINS);
    DCMotorIO frDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.SIM_DRIVE_GAINS);
    DCMotorIO frSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.SIM_STEER_GAINS);
    DCMotorIO blDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.SIM_DRIVE_GAINS);
    DCMotorIO blSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.SIM_STEER_GAINS);
    DCMotorIO brDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.SIM_DRIVE_GAINS);
    DCMotorIO brSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.SIM_STEER_GAINS);

    GyroIO gyroIO = new GyroIO() {};
    WheeledOdometrySimThread thread =
        new WheeledOdometrySimThread(
            () -> flDriveIO.getAppliedPosition(),
            () -> flSteerIO.getAppliedPosition(),
            () -> blDriveIO.getAppliedPosition(),
            () -> blSteerIO.getAppliedPosition(),
            () -> brDriveIO.getAppliedPosition(),
            () -> brSteerIO.getAppliedPosition(),
            () -> frDriveIO.getAppliedPosition(),
            () -> frSteerIO.getAppliedPosition());

    return new Swerve(
        flDriveIO, flSteerIO, blDriveIO, blSteerIO, brDriveIO, brSteerIO, frDriveIO, frSteerIO,
        gyroIO, thread);
  }

  public static Swerve createReal() {
    UnitConverter driveRatioConverter =
        UnitConverter.scale(2 * Math.PI * SwerveConfig.WHEEL_RADIUS_METER).withUnits("rot", "m");
    UnitConverter steerRatioConverter = UnitConverter.scale(2 * Math.PI).withUnits("rot", "rad");

    DCMotorIOTalonfx flDriveIO =
        new DCMotorIOTalonfx(
            "flDrive",
            Ports.Can.FL_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonfxCancoder flSteerIO =
        new DCMotorIOTalonfxCancoder(
            "flSteer",
            Ports.Can.FL_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.FL_STEER_SENSOR,
            SwerveConfig.FL_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.FL_CANCODER_OFFSET).withUnits("rot", "rot"));

    DCMotorIOTalonfx frDriveIO =
        new DCMotorIOTalonfx(
            "frDrive",
            Ports.Can.FR_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonfxCancoder frSteerIO =
        new DCMotorIOTalonfxCancoder(
            "frSteer",
            Ports.Can.FR_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.FR_STEER_SENSOR,
            SwerveConfig.FR_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.FR_CANCODER_OFFSET).withUnits("rot", "rot"));

    DCMotorIOTalonfx blDriveIO =
        new DCMotorIOTalonfx(
            "blDrive",
            Ports.Can.BL_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonfxCancoder blSteerIO =
        new DCMotorIOTalonfxCancoder(
            "blSteer",
            Ports.Can.BL_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.BL_STEER_SENSOR,
            SwerveConfig.BL_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.BL_CANCODER_OFFSET).withUnits("rot", "rot"));

    DCMotorIOTalonfx brDriveIO =
        new DCMotorIOTalonfx(
            "brDrive",
            Ports.Can.BR_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonfxCancoder brSteerIO =
        new DCMotorIOTalonfxCancoder(
            "brSteer",
            Ports.Can.BR_STEER_MOTOR,
            SwerveConfig.getX2SteerTalonNoEncoderConfig(),
            Ports.Can.BR_STEER_SENSOR,
            SwerveConfig.BR_CANCODER_CONFIG,
            steerRatioConverter,
            UnitConverter.offset(SwerveConfig.BR_CANCODER_OFFSET).withUnits("rot", "rot"));

    GyroIOPigeon2 gyroIO = new GyroIOPigeon2(Ports.Can.CHASSIS_PIGEON);

    WheeledOdometryPhoenixThread thread =
        new WheeledOdometryPhoenixThread(
            flDriveIO.getRawPositionSignal(),
            flSteerIO.getRawPositionSignal(),
            blDriveIO.getRawPositionSignal(),
            blSteerIO.getRawPositionSignal(),
            brDriveIO.getRawPositionSignal(),
            brSteerIO.getRawPositionSignal(),
            frDriveIO.getRawPositionSignal(),
            frSteerIO.getRawPositionSignal(),
            gyroIO.getYawSignal(),
            SwerveConfig.ODOMETRY_FREQUENCY_HZ,
            SwerveConfig.WHEEL_RADIUS_METER);

    return new Swerve(
        flDriveIO, flSteerIO, blDriveIO, blSteerIO, brDriveIO, brSteerIO, frDriveIO, frSteerIO,
        gyroIO, thread);
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
        new GyroIO() {},
        new WheeledOdometryThread() {});
  }

  private Swerve(
      DCMotorIO flDriveIO,
      DCMotorIO flSteerIO,
      DCMotorIO blDriveIO,
      DCMotorIO blSteerIO,
      DCMotorIO brDriveIO,
      DCMotorIO brSteerIO,
      DCMotorIO frDriveIO,
      DCMotorIO frSteerIO,
      GyroIO gyroIO,
      WheeledOdometryThread odometryThread) {

    modules[0] = new SwerveModule(flDriveIO, flSteerIO, "ModuleFL");
    modules[1] = new SwerveModule(blDriveIO, blSteerIO, "ModuleBL");
    modules[2] = new SwerveModule(brDriveIO, brSteerIO, "ModuleBR");
    modules[3] = new SwerveModule(frDriveIO, frSteerIO, "ModuleFR");
    odometry = new SwerveOdometry(gyroIO, odometryThread.start());
  }

  public void registerVisualize(VisualizeService visualizer) {
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_FL,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                SwerveConfig.FL_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[0].getState().angle.getRadians(), 0)));
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_BL,
        -1,
        () ->
            new Transform3d(
                SwerveConfig.BL_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[1].getState().angle.getRadians(), 0)));
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_BR,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                SwerveConfig.BR_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[2].getState().angle.getRadians(), 0)));
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_FR,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                SwerveConfig.FR_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[3].getState().angle.getRadians(), 0)));
  }
}
