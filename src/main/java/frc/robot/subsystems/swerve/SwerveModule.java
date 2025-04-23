package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drivers.dcmotor.*;
import frc.robot.utils.AlertUtil;
import frc.robot.utils.GainsUtil.PdsGains;
import frc.robot.utils.LoggedTunableGains;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final LoggedTunableGains<PdsGains> driveGains;
  private static final LoggedTunableGains<PdsGains> steerGains;

  static {
    driveGains =
        new LoggedTunableGains<PdsGains>("Swerve/Module/DriveGains", SwerveConfig.DRIVE_GAINS);
    steerGains =
        new LoggedTunableGains<PdsGains>("Swerve/Module/SteerGains", SwerveConfig.STEER_GAINS);
  }

  private final String name;

  private final DCMotorIO driveIO;
  private final DCMotorIOInputsAutoLogged driveInputs = new DCMotorIOInputsAutoLogged();
  private final AlertUtil driveMotorOfflineAlert;

  private final DCMotorIO steerIO;
  private final DCMotorIOInputsAutoLogged steerInputs = new DCMotorIOInputsAutoLogged();
  private final AlertUtil steerMotorOfflineAlert;

  public SwerveModule(DCMotorIO driveIO, DCMotorIO steerIO, String name) {
    this.driveIO = driveIO;
    this.steerIO = steerIO;
    this.steerIO.setRotationContinuous(true);
    this.name = name;

    driveMotorOfflineAlert =
        new AlertUtil(this.name + " drive motor offline!", AlertUtil.AlertType.WARNING);
    steerMotorOfflineAlert =
        new AlertUtil(this.name + " steer motor offline!", AlertUtil.AlertType.WARNING);

    stop();
  }

  public void updateInputs() {
    driveIO.updateInputs(driveInputs);
    steerIO.updateInputs(steerInputs);

    Logger.processInputs("Swerve/" + name + "/Drive", driveInputs);
    Logger.processInputs("Swerve/" + name + "/Steer", steerInputs);

    LoggedTunableGains.ifChanged(hashCode(), () -> driveIO.setPidsg(driveGains.get()), driveGains);
    LoggedTunableGains.ifChanged(hashCode(), () -> steerIO.setPidsg(steerGains.get()), steerGains);

    driveMotorOfflineAlert.set(!driveInputs.connected);
    steerMotorOfflineAlert.set(!steerInputs.connected);
  }

  void setState(SwerveModuleState state, double velocityFeedforward) {

    driveIO.setAppliedVelocityF(
        state.speedMetersPerSecond, velocityFeedforward / SwerveConfig.DRIVE_FF_KT);
    steerIO.setAppliedPosition(state.angle.getRadians());
  }

  void setState(SwerveModuleState state) {
    setState(state, 0.0);
  }

  SwerveModuleState getState() {
    return new SwerveModuleState(
        driveInputs.appliedVelocity, Rotation2d.fromRadians(steerInputs.appliedPosition));
  }

  SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveInputs.appliedPosition, Rotation2d.fromRadians(steerInputs.appliedPosition));
  }

  void stop() {
    driveIO.stop();
    steerIO.stop();
  }
}
