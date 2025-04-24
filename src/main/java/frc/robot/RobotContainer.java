package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.TeleopHeaderController;
import frc.robot.subsystems.vision.AtagVision;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;

public class RobotContainer {
  // physical subsystems
  private final Swerve swerve;
  private final AtagVision vision;

  private final Odometry odometry = new Odometry();

  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);

  public RobotContainer() {
    if (Robot.isReal()) {
      // physical subsystems
      swerve = Swerve.createReal();
      vision = AtagVision.createReal();
    } else if (Robot.isSimulation()) {
      // simulation subsystems
      swerve = Swerve.createSim();
      vision = AtagVision.createSim(() -> swerve.getPose());
    } else {
      // dummy subsystems
      swerve = Swerve.createIO();
      vision = AtagVision.createIO();
    }

    configureOdometry();
    configureJoystick();
    configureVisualizer();
  }

  private void configureSubsystems() {}

  private void configureJoystick() {
    swerve.setDefaultCommand(
        swerve.registerControllerCommand(
            new TeleopHeaderController(
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX())));

    joystick.a().onTrue(swerve.resetGyroHeadingCommand(new Rotation2d()));
    joystick
        .y()
        .onTrue(swerve.registerBetterPoseCommandSupplier(() -> new UncertainPose2d(new Pose2d())));
    joystick.x().onTrue(joystickHumbleCommand(0.3));
  }

  private void configureOdometry() {
    // odometry.registerObservation(
    // new Odometry.PoseObservation(
    // "Wheeled",
    // swerve::getWheeledPose,
    // swerve::getWheeledPoseCovariance,
    // () -> Timer.getFPGATimestamp()));

    // // 注册AtagVision的位姿估计
    // odometry.registerObservation(
    // new Odometry.PoseObservation(
    // "AtagVision",
    // () -> vision.getEstimatedPose(),
    // () -> vision.getPoseCovariance(),
    // () -> vision.getLatestTimestamp()));
  }

  private void configureVisualizer() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  private Command joystickHumbleCommand(double seconds) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds)
        .withName("[Joystick] Rumble " + Math.round(seconds * 10) / 10.0 + "s");
  }
}
