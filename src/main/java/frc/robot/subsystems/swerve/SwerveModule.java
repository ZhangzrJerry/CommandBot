package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.dcmotor.DCMotorIO;
import frc.robot.utils.Alert;
import frc.robot.utils.LoggedTunableNumber;

public class SwerveModule extends SubsystemBase {
  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("Swerve/Module/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("Swerve/Module/DriveKd");
  private static final LoggedTunableNumber driveKs = new LoggedTunableNumber("Swerve/Module/DriveKs");
  private static final LoggedTunableNumber steerKp = new LoggedTunableNumber("Swerve/Module/SteerKp");
  private static final LoggedTunableNumber steerKd = new LoggedTunableNumber("Swerve/Module/SteerKd");
  private static final LoggedTunableNumber steerKs = new LoggedTunableNumber("Swerve/Module/SteerKs");

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

  private final DCMotorIO driveIO;
  private final DCMotorIOAutoLogged driveInputs = new DCMotorIOAutoLogged();
  private final Alert driveMotorOfflineAlert;

  private final DCMotorIO steerIO;
  private final DCMotorIOAutoLogged steerInputs = new DCMotorIOAutoLogged();
  private final Alert steerMotorOfflineAlert;

  public SwerveModule(DCMotorIO driveIO, DCMotorIO steerIO, String name) {
    this.driveIO = driveIO;
    this.steerIO = steerIO;
    this.name = name;

    driveMotorOfflineAlert = new Alert(this.name + " drive motor offline!", Alert.AlertType.WARNING);
    steerMotorOfflineAlert = new Alert(this.name + " steer motor offline!", Alert.AlertType.WARNING);
  }

  @Override
  public void periodic() {
    driveIO.updateInputs(driveInputs);
    steerIO.updateInputs(steerInputs);
    Logger.processInputs("Swerve/" + name + "/Drive", driveInputs);
    Logger.processInputs("Swerve/" + name + "/Steer", steerInputs);

    LoggedTunableNumber.ifChanged(hashCode(),
        () -> driveIO.setPIDF(driveKp.get(), 0.0, driveKd.get(), driveKs.get(), 0.0), driveKp, driveKd, driveKs);
    LoggedTunableNumber.ifChanged(hashCode(),
        () -> steerIO.setPIDF(steerKp.get(), 0.0, steerKd.get(), steerKs.get(), 0.0), steerKp, steerKd, steerKs);

    driveMotorOfflineAlert.set(!driveInputs.connected);
    steerMotorOfflineAlert.set(!steerInputs.connected);

  }
}
