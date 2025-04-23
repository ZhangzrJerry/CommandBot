package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.TeleopHeaderController;
import frc.robot.virtuals.odometry.Odometry;

public class RobotContainer {
  // physical subsystems
  private final Swerve swerve;

  // virtual subsystems
  private final CommandXboxController joystick = new CommandXboxController(Ports.Joystick.DRIVER);
  private final Odometry odometry = new Odometry();

  public RobotContainer() {
    if (Robot.isReal()) {
      // physical subsystems
      swerve = Swerve.createReal();
    } else if (Robot.isSimulation()) {
      // simulation subsystems
      swerve = Swerve.createSim();
    } else {
      // dummy subsystems
      swerve = Swerve.createIO();
    }

    configureOdometry();
    configureJoystick();
    configureVisualizer();
  }

  private void configureSubsystems() {}

  private void configureJoystick() {
    swerve.setDefaultCommand(
        swerve.setControllerCommand(
            new TeleopHeaderController(
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX())));

    joystick.a().onTrue(swerve.resetGyroHeadingCommand(new Rotation2d()));
    joystick
        .b()
        .onTrue(
            swerve.resetWheeledPoseCommand(
                new Pose2d(), new Matrix<N3, N3>(N3.instance, N3.instance)));
    joystick.x().onTrue(joystickHumbleCommand(0.3));
  }

  private void configureOdometry() {
    odometry.registerObservation(
        new Odometry.PoseObservation(
            "Wheeled",
            swerve::getWheeledPose,
            swerve::getWheeledPoseCovariance,
            () -> Timer.getFPGATimestamp()));
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
