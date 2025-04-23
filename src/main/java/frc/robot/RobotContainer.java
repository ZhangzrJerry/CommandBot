package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.virtuals.odometry.Odometry;

public class RobotContainer {
  // real subsystems
  private final Swerve swerve;

  // virtual subsystems
  private final CommandXboxController joystick = new CommandXboxController(Ports.Joystick.DRIVER);
  private final Odometry odometry = new Odometry();

  // command

  public RobotContainer() {
    // real subsystems
    if (Robot.isReal()) {
      swerve = Swerve.createReal();
    } else if (Robot.isSimulation()) {
      swerve = Swerve.createSim();
    } else {
      swerve = Swerve.createIO();
    }

    configListener();
    configSpeaker();
  }

  private void configListener() {
    odometry.registerObservation(
        new Odometry.PoseObservation(
            "wheel",
            swerve::getWheeledPose,
            swerve::getWheeledPoseCovariance,
            () -> Timer.getFPGATimestamp()));
  }

  private void configSpeaker() {
    // swerve
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
