package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.joystick.JoystickCommands;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveController;

public class CompositeCommands {
  private final SwerveCommands swerveCommands;
  private final JoystickCommands joystickCommands;
  private final Swerve swerve;

  public CompositeCommands(
      SwerveCommands swerveCommands, JoystickCommands joystickCommands, Swerve swerve) {
    this.swerveCommands = swerveCommands;
    this.joystickCommands = joystickCommands;
    this.swerve = swerve;
  }

  // 创建自动路径点命令（包含震动反馈）
  public Command createAutoWaypointCommand() {
    return Commands.sequence(
            swerveCommands.createWaypointCommand(), joystickCommands.createRumbleCommand(0.5))
        .withName("[Composite] Auto Waypoint with Feedback");
  }

  // 创建紧急停止命令（停止所有运动并震动）
  public Command createEmergencyStopCommand() {
    return Commands.parallel(
            Commands.runOnce(() -> swerve.setController(new SwerveController() {})),
            joystickCommands.createRumbleCommand(1.0))
        .withName("[Composite] Emergency Stop");
  }
}
