package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
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

    bindingCommands();
  }

  private void bindingCommands() {
    swerve.setDefaultCommand(
        swerve.registerControllerCommand(
            new TeleopHeaderController(
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX())));

    joystick.a().onTrue(swerve.resetGyroHeadingCommand(new Rotation2d()));
    joystick.x().onTrue(joystickRumbleCommand(0.3));
  }

  public Command getInitializationCommand() {
    return Commands.sequence(
            // swerve
            // swerve.registerBetterPoseCommandSupplier(() -> odometry.getEstimatedPose()),
            // odometry
            odometry.registerObservation(
                new Odometry.PoseObservation(
                    "Wheeled", swerve::getUncertainPose2d, () -> Timer.getFPGATimestamp(), 0.8)),
            odometry.registerObservation(
                new Odometry.PoseObservation(
                    "AtagVision", () -> vision.getLatestPose(), () -> vision.getLatestTimestamp())))
        .withName("### Robot Initialization ...")
        .ignoringDisable(true);
  }

  public Command getAutonomousCommand() {
    return Commands.none().withName("### Robot Autonomous ...");
  }

  public Command getSimulationConfigureCommand() {
    return Commands.sequence(
            swerve.resetWheeledPoseCommand(
                new UncertainPose2d(new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(0)))))
        .withName("### Robot Simulation Configure ...")
        .ignoringDisable(true);
  }

  private Command joystickRumbleCommand(double seconds) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds)
        .withName("[Joystick] Rumble " + Math.round(seconds * 10) / 10.0 + "s");
  }
}
