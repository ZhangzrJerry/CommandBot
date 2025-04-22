package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drivers.dcmotor.*;
import frc.robot.utils.Alert;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Swerve/Module/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("Swerve/Module/DriveKd");
  private static final LoggedTunableNumber driveKs =
      new LoggedTunableNumber("Swerve/Module/DriveKs");
  private static final LoggedTunableNumber steerKp =
      new LoggedTunableNumber("Swerve/Module/SteerKp");
  private static final LoggedTunableNumber steerKd =
      new LoggedTunableNumber("Swerve/Module/SteerKd");
  private static final LoggedTunableNumber steerKs =
      new LoggedTunableNumber("Swerve/Module/SteerKs");

  static {
    var driveSlot = SwerveConfig.getX2DriveTalonConfig().Slot0;
    driveKp.initDefault(driveSlot.kP);
    driveKd.initDefault(driveSlot.kD);
    driveKs.initDefault(driveSlot.kS);

    var steerSlot = SwerveConfig.getX2SteerTalonNoEncoderConfig().Slot0;
    steerKp.initDefault(steerSlot.kP);
    steerKd.initDefault(steerSlot.kD);
    steerKs.initDefault(steerSlot.kS);
  }

  private final String name;
  private SwerveModuleState state;
  private double velocityFeedforward = 0.0;

  private final DCMotorIO driveIO;
  private final DCMotorIOInputsAutoLogged driveInputs = new DCMotorIOInputsAutoLogged();
  private final Alert driveMotorOfflineAlert;

  private final DCMotorIO steerIO;
  private final DCMotorIOInputsAutoLogged steerInputs = new DCMotorIOInputsAutoLogged();
  private final Alert steerMotorOfflineAlert;

  public SwerveModule(DCMotorIO driveIO, DCMotorIO steerIO, String name) {
    this.driveIO = driveIO;
    this.steerIO = steerIO;
    this.name = name;

    driveMotorOfflineAlert =
        new Alert(this.name + " drive motor offline!", Alert.AlertType.WARNING);
    steerMotorOfflineAlert =
        new Alert(this.name + " steer motor offline!", Alert.AlertType.WARNING);

    stop();
  }

  public void update() {
    driveIO.updateInputs(driveInputs);
    steerIO.updateInputs(steerInputs);

    Logger.processInputs("Swerve/" + name + "/Drive", driveInputs);
    Logger.processInputs("Swerve/" + name + "/Steer", steerInputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> driveIO.setPIDF(driveKp.get(), 0.0, driveKd.get(), driveKs.get(), 0.0),
        driveKp,
        driveKd,
        driveKs);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> steerIO.setPIDF(steerKp.get(), 0.0, steerKd.get(), steerKs.get(), 0.0),
        steerKp,
        steerKd,
        steerKs);

    driveMotorOfflineAlert.set(!driveInputs.connected);
    steerMotorOfflineAlert.set(!steerInputs.connected);

    driveIO.setVelocityF(
        state.speedMetersPerSecond, velocityFeedforward / SwerveConfig.DRIVE_FF_KT);
    steerIO.setPosition(state.angle.getRadians());
  }

  void setState(SwerveModuleState state, double velocityFeedforward) {
    this.state = state;
    this.velocityFeedforward = velocityFeedforward;
  }

  void setState(SwerveModuleState state) {
    setState(state, 0.0);
  }

  SwerveModuleState getState() {
    return new SwerveModuleState(
        driveInputs.velocity, Rotation2d.fromRadians(steerInputs.position));
  }

  SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveInputs.position, Rotation2d.fromRadians(steerInputs.position));
  }

  void stop() {
    this.state = new SwerveModuleState(0, Rotation2d.fromRadians(steerInputs.position));
  }
}
