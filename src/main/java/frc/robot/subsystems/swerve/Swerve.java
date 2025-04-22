package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.drivers.dcmotor.DCMotorIO;
import frc.robot.drivers.dcmotor.DCMotorIOKraken;
import frc.robot.drivers.dcmotor.DCMotorIOKrakenCancoder;
import frc.robot.drivers.dcmotor.DCMotorIOSim;
import frc.robot.utils.UnitConverter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4];
  @Setter private SwerveController controller = new SwerveController() {};

  public Swerve() {
    DCMotorIO flDriveIO,
        flSteerIO,
        frDriveIO,
        frSteerIO,
        blDriveIO,
        blSteerIO,
        brDriveIO,
        brSteerIO;
    if (Robot.isReal()) {
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
    } else if (Robot.isSimulation()) {
      UnitConverter driveRatioConverter =
          UnitConverter.scale(SwerveConfig.WHEEL_RADIUS_METER / SwerveConfig.DRIVE_REDUCTION)
              .withUnits("rad", "m");
      UnitConverter steerRatioConverter = UnitConverter.identity().withUnits("rad", "rad");
      flDriveIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.025, SwerveConfig.DRIVE_REDUCTION, driveRatioConverter);
      flSteerIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.004, SwerveConfig.STEER_REDUCTION, steerRatioConverter);
      frDriveIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.025, SwerveConfig.DRIVE_REDUCTION, driveRatioConverter);
      frSteerIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.004, SwerveConfig.STEER_REDUCTION, steerRatioConverter);
      blDriveIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.025, SwerveConfig.DRIVE_REDUCTION, driveRatioConverter);
      blSteerIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.004, SwerveConfig.STEER_REDUCTION, steerRatioConverter);
      brDriveIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.025, SwerveConfig.DRIVE_REDUCTION, driveRatioConverter);
      brSteerIO =
          new DCMotorIOSim(
              DCMotor.getKrakenX60(1), 0.004, SwerveConfig.STEER_REDUCTION, steerRatioConverter);
    } else {
      flDriveIO = new DCMotorIO() {};
      flSteerIO = new DCMotorIO() {};
      frDriveIO = new DCMotorIO() {};
      frSteerIO = new DCMotorIO() {};
      blDriveIO = new DCMotorIO() {};
      blSteerIO = new DCMotorIO() {};
      brDriveIO = new DCMotorIO() {};
      brSteerIO = new DCMotorIO() {};
    }

    modules[0] = new SwerveModule(flDriveIO, flSteerIO, "FL");
    modules[1] = new SwerveModule(frDriveIO, frSteerIO, "FR");
    modules[2] = new SwerveModule(blDriveIO, blSteerIO, "BL");
    modules[3] = new SwerveModule(brDriveIO, brSteerIO, "BR");
  }

  @Override
  public void periodic() {
    ChassisSpeeds speeds = controller.getChassisSpeeds();
    Logger.recordOutput("Swerve/ControllerChassisSpeeds", speeds);
    SwerveModuleState[] states = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }
}
