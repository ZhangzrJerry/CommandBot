package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.interfaces.hardwares.motors.*;
import frc.robot.utils.dashboard.TunableNumbers;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final String name;

  private final DCMotorIO driveIO;
  private final DCMotorIOInputsAutoLogged driveInputs = new DCMotorIOInputsAutoLogged();

  private final DCMotorIO steerIO;
  private final DCMotorIOInputsAutoLogged steerInputs = new DCMotorIOInputsAutoLogged();

  public SwerveModule(DCMotorIO driveIO, DCMotorIO steerIO, String name) {
    this.driveIO = driveIO;
    this.steerIO = steerIO;
    this.steerIO.setRotationContinuous(true);
    this.name = name;

    stop();
  }

  public void updateInputs() {
    driveIO.updateInputs(driveInputs);
    steerIO.updateInputs(steerInputs);

    Logger.processInputs("Swerve/" + name + "/Drive", driveInputs);
    Logger.processInputs("Swerve/" + name + "/Steer", steerInputs);

    TunableNumbers.ifChanged(
        hashCode(), () -> driveIO.setGains(SwerveConfig.DRIVE_GAINS), SwerveConfig.DRIVE_GAINS);
    TunableNumbers.ifChanged(
        hashCode(), () -> steerIO.setGains(SwerveConfig.STEER_GAINS), SwerveConfig.STEER_GAINS);
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

  void setDriveCharacterizationCurrent(double currentAmp) {
    driveIO.setCurrent(currentAmp);
    steerIO.setAppliedPosition(0.0);
  }

  void stop() {
    driveIO.stop();
    steerIO.stop();
  }
}
