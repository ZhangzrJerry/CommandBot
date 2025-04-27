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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.commands.CharacterizationCommand;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonFX;
import frc.robot.interfaces.hardwares.sensors.gyro.GyroIO;
import frc.robot.interfaces.hardwares.sensors.gyro.GyroIOPigeon2;
import frc.robot.interfaces.services.PoseService;
import frc.robot.interfaces.threads.wheeled.WheeledOdometryPhoenixThread;
import frc.robot.interfaces.threads.wheeled.WheeledOdometrySimThread;
import frc.robot.interfaces.threads.wheeled.WheeledOdometryThread;
import frc.robot.services.TransformTree;
import frc.robot.utils.math.EqualsUtil;
import frc.robot.utils.math.GeomUtil;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import frc.robot.utils.math.UnitConverter;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class, EqualsUtil.GeomExtensions.class})
public class Swerve extends SubsystemBase {
  @Getter private boolean isKsCharacterization = false;
  @Getter private boolean isWheelRadiusCharacterization = false;
  private final SwerveModule[] modules = new SwerveModule[4];
  @Getter private final SwerveOdometry odometry;
  @Setter private SwerveController controller = new SwerveController() {};

  @Setter
  @AutoLogOutput(key = "Swerve/customMaxTiltAccelScale")
  private DoubleSupplier customMaxTiltAccelScale = () -> 1.0;

  private SwerveModuleState[] lastGoalModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  @AutoLogOutput(key = "Swerve/goalWheelRadiusCharacterizationAngularVel")
  private double goalWheelRadiusCharacterizationAngularVel = 0.0;

  public Command registerControllerCmd(SwerveController controller) {
    return Commands.run(
            () -> {
              this.controller = controller;
            },
            this)
        .withName("Swerve/Register Controller: " + controller.getName());
  }

  public Command resetWheeledPoseCmd(UncertainPose2d pose) {
    return Commands.runOnce(
            () -> {
              odometry.resetWheeledPose(pose);
            })
        .withName("Swerve/Reset Wheeled Pose");
  }

  public Command resetGyroHeadingCmd(Rotation2d yaw) {
    return Commands.runOnce(
            () -> {
              odometry.resetGyroHeading(yaw);
            })
        .withName("Swerve/Reset Gyro Heading");
  }

  public Pose2d getPose() {
    return odometry.getPose().getPose();
  }

  public UncertainPose2d getUncertainPose2d() {
    return odometry.getPose();
  }

  public Boolean atGoal() {
    return controller.headingAtGoal() && controller.translationAtGoal();
  }

  public Boolean atToleranceGoal() {
    return controller.headingAtGoal() && controller.translationErrorWithin();
  }

  public Boolean atToleranceGoal(double tolerance) {
    return controller.translationErrorWithin(tolerance);
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

  public static Swerve createSim(PoseService poseService) {
    UnitConverter driveRatioConverter =
        UnitConverter.scale(SwerveConfig.WHEEL_RADIUS_METER).withUnits("rad", "m");
    UnitConverter steerRatioConverter = UnitConverter.identity().withUnits("rad", "rad");

    DCMotorIO flDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    DCMotorIO flSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);
    DCMotorIO frDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    DCMotorIO frSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);
    DCMotorIO blDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    DCMotorIO blSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);
    DCMotorIO brDriveIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.025,
            SwerveConfig.DRIVE_REDUCTION,
            driveRatioConverter,
            SwerveConfig.DRIVE_GAINS);
    DCMotorIO brSteerIO =
        new DCMotorIOSim(
            DCMotor.getKrakenX60(1),
            0.004,
            SwerveConfig.STEER_REDUCTION,
            steerRatioConverter,
            SwerveConfig.STEER_GAINS);

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
        flDriveIO,
        flSteerIO,
        blDriveIO,
        blSteerIO,
        brDriveIO,
        brSteerIO,
        frDriveIO,
        frSteerIO,
        gyroIO,
        thread,
        poseService);
  }

  public static Swerve createReal(PoseService poseService) {
    UnitConverter driveRatioConverter =
        UnitConverter.scale(2 * Math.PI * SwerveConfig.WHEEL_RADIUS_METER).withUnits("rot", "m");
    UnitConverter steerRatioConverter = UnitConverter.scale(2 * Math.PI).withUnits("rot", "rad");

    DCMotorIOTalonFX flDriveIO =
        new DCMotorIOTalonFX(
            "FL Drive",
            Ports.Can.FL_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonFX flSteerIO =
        new DCMotorIOTalonFX(
                "FL Steer",
                Ports.Can.FL_STEER_MOTOR,
                SwerveConfig.getX2SteerTalonNoEncoderConfig(),
                steerRatioConverter,
                UnitConverter.offset(2 * Math.PI * SwerveConfig.FL_CANCODER_OFFSET)
                    .withUnits("rad", "rad"))
            .withCancoder("FLSteer", Ports.Can.FL_STEER_SENSOR, SwerveConfig.FL_CANCODER_CONFIG);

    DCMotorIOTalonFX frDriveIO =
        new DCMotorIOTalonFX(
            "FR Drive",
            Ports.Can.FR_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonFX frSteerIO =
        new DCMotorIOTalonFX(
                "FR Steer",
                Ports.Can.FR_STEER_MOTOR,
                SwerveConfig.getX2SteerTalonNoEncoderConfig(),
                steerRatioConverter,
                UnitConverter.offset(2 * Math.PI * SwerveConfig.FR_CANCODER_OFFSET)
                    .withUnits("rad", "rad"))
            .withCancoder("FRSteer", Ports.Can.FR_STEER_SENSOR, SwerveConfig.FR_CANCODER_CONFIG);

    DCMotorIOTalonFX blDriveIO =
        new DCMotorIOTalonFX(
            "BL Drive",
            Ports.Can.BL_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonFX blSteerIO =
        new DCMotorIOTalonFX(
                "BL Steer",
                Ports.Can.BL_STEER_MOTOR,
                SwerveConfig.getX2SteerTalonNoEncoderConfig(),
                steerRatioConverter,
                UnitConverter.offset(2 * Math.PI * SwerveConfig.BL_CANCODER_OFFSET)
                    .withUnits("rad", "rad"))
            .withCancoder("BLSteer", Ports.Can.BL_STEER_SENSOR, SwerveConfig.BL_CANCODER_CONFIG);

    DCMotorIOTalonFX brDriveIO =
        new DCMotorIOTalonFX(
            "BR Drive",
            Ports.Can.BR_DRIVE_MOTOR,
            SwerveConfig.getX2DriveTalonConfig(),
            driveRatioConverter);
    DCMotorIOTalonFX brSteerIO =
        new DCMotorIOTalonFX(
                "BR Steer",
                Ports.Can.BR_STEER_MOTOR,
                SwerveConfig.getX2SteerTalonNoEncoderConfig(),
                steerRatioConverter,
                UnitConverter.offset(2 * Math.PI * SwerveConfig.BR_CANCODER_OFFSET)
                    .withUnits("rad", "rad"))
            .withCancoder("BRSteer", Ports.Can.BR_STEER_SENSOR, SwerveConfig.BR_CANCODER_CONFIG);

    GyroIOPigeon2 gyroIO = new GyroIOPigeon2("Chassis Gyro", Ports.Can.CHASSIS_PIGEON);

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
        flDriveIO,
        flSteerIO,
        blDriveIO,
        blSteerIO,
        brDriveIO,
        brSteerIO,
        frDriveIO,
        frSteerIO,
        gyroIO,
        thread,
        poseService);
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
        new WheeledOdometryThread() {},
        new PoseService() {});
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
      WheeledOdometryThread odometryThread,
      PoseService poseService) {

    modules[0] = new SwerveModule(flDriveIO, flSteerIO, "ModuleFL");
    modules[1] = new SwerveModule(blDriveIO, blSteerIO, "ModuleBL");
    modules[2] = new SwerveModule(brDriveIO, brSteerIO, "ModuleBR");
    modules[3] = new SwerveModule(frDriveIO, frSteerIO, "ModuleFR");
    odometry = new SwerveOdometry(gyroIO, odometryThread.start(), poseService);
  }

  public void registerTransform(TransformTree transformTree) {
    transformTree.registerTransformComponent(
        Constants.Ascope.Component.SWERVE_FL,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                SwerveConfig.FL_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[0].getState().angle.getRadians(), 0)));
    transformTree.registerTransformComponent(
        Constants.Ascope.Component.SWERVE_BL,
        -1,
        () ->
            new Transform3d(
                SwerveConfig.BL_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[1].getState().angle.getRadians(), 0)));
    transformTree.registerTransformComponent(
        Constants.Ascope.Component.SWERVE_BR,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                SwerveConfig.BR_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[2].getState().angle.getRadians(), 0)));
    transformTree.registerTransformComponent(
        Constants.Ascope.Component.SWERVE_FR,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                SwerveConfig.FR_ZEROED_TF.getTranslation(),
                new Rotation3d(0, modules[3].getState().angle.getRadians(), 0)));
  }

  public double getModuleDriveCharacterizationVel() {
    var sumDriveVelocity = 0.0;
    for (var module : modules) {
      sumDriveVelocity += Math.abs(module.getDriveVelMeterPerSec());
    }
    return sumDriveVelocity / 4.0;
  }

  public void setModuleDriveCharacterizationCurrent(double currentAmp) {
    isKsCharacterization = true;
    isWheelRadiusCharacterization = false;
    for (var module : modules) {
      module.setDriveCharacterizationCurrent(currentAmp);
    }
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(SwerveModule::getDrivePositionRad).toArray();
  }

  public Command getKsCharacterizationCmd() {
    return CharacterizationCommand.createKsCharacterizationCommand(
        "Swerve",
        () -> SwerveConfig.currentRampFactor.get(),
        () -> SwerveConfig.minVelocity.get(),
        current -> {
          isKsCharacterization = true;
          isWheelRadiusCharacterization = false;
          for (var module : modules) {
            module.setDriveCharacterizationCurrent(current);
          }
        },
        this::getModuleDriveCharacterizationVel);
  }

  public Command getWheelRadiusCharacterizationCmd(Direction rotateDirection) {
    return CharacterizationCommand.createWheelRadiusCharacterizationCommand(
        "Swerve",
        () ->
            rotateDirection.value
                * Units.degreesToRadians(SwerveConfig.characterizationSpeedDegreePerSec.get()),
        angularVel -> {
          isKsCharacterization = false;
          isWheelRadiusCharacterization = true;
          goalWheelRadiusCharacterizationAngularVel = angularVel;
        },
        () -> odometry.getPose().getPose().getRotation().getRadians(),
        () -> {
          var sumWheelPosition = 0.0;
          var wheelPositions = getWheelRadiusCharacterizationPosition();
          for (int i = 0; i < 4; i++) {
            sumWheelPosition += Math.abs(wheelPositions[i]);
          }
          return sumWheelPosition / 4.0;
        },
        () -> SwerveConfig.WHEELBASE_DIAGONAL_METER,
        () -> {
          isKsCharacterization = false;
          isWheelRadiusCharacterization = false;
          goalWheelRadiusCharacterizationAngularVel = 0.0;
        });
  }

  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;

    Direction(int value) {
      this.value = value;
    }
  }
}
