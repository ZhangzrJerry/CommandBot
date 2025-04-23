package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.AlongWaypointsController;
import frc.robot.subsystems.swerve.controller.TeleopHeadlessController;
import frc.robot.virtuals.odometry.Odometry;

public class SwerveCommands {
  private final Swerve swerve;
  private final Odometry odometry;
  private final CommandXboxController joystick;

  public SwerveCommands(Swerve swerve, Odometry odometry, CommandXboxController joystick) {
    this.swerve = swerve;
    this.odometry = odometry;
    this.joystick = joystick;
  }

  // 创建无头控制命令
  public Command createHeadlessControlCommand() {
    return Commands.run(
            () ->
                swerve.setController(
                    new TeleopHeadlessController(
                        () -> -joystick.getLeftY(),
                        () -> -joystick.getLeftX(),
                        () -> -joystick.getRightX(),
                        () -> odometry.getGyroYaw())),
            swerve)
        .withName("[Swerve] Headless Control");
  }

  // 创建路径点命令
  public Command createWaypointCommand() {
    return Commands.run(
            () ->
                swerve.setController(
                    new AlongWaypointsController(
                        () -> odometry.getEstimatedPose(), new Pose2d(0, 0, new Rotation2d()))),
            swerve)
        .withName("## Proceed to Waypoint");
  }
}
