package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveController;

public class SwerveCommands extends Command {
  private final Swerve swerve;

  public SwerveCommands(Swerve swerve) {
    this.swerve = swerve;
  }

  public Command setController(SwerveController controller) {
    return Commands.run(
        () -> {
          swerve.setController(controller);
        },
        swerve)
        .withName("[Swerve] " + controller.getName());
  }
}
