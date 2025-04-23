package frc.robot.commands.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class JoystickCommands {
  private final CommandXboxController joystick;

  public JoystickCommands(CommandXboxController joystick) {
    this.joystick = joystick;
  }

  // 创建震动命令
  public Command createRumbleCommand(double intensity) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, intensity),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withName("[Joystick] Rumble");
  }
}
