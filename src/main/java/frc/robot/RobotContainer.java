package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.TeleopHeaderController;
import frc.robot.subsystems.vision.AtagVision;

public class RobotContainer {
  // physical subsystems
  private final Swerve swerve;
  private final Arm arm;
  private final AtagVision vision;

  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);

  public RobotContainer() {
    if (Robot.isReal()) {
      // physical subsystems
      swerve = Swerve.createReal();
      arm = Arm.createReal();
      vision = AtagVision.createReal();
    } else if (Robot.isSimulation()) {
      // simulation subsystems
      swerve = Swerve.createSim();
      arm = Arm.createSim();
      vision = AtagVision.createSim(() -> swerve.getPose());
    } else {
      // dummy subsystems
      swerve = Swerve.createIO();
      arm = Arm.createIO();
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

    joystick.a().onTrue(arm.getElbowKsCharacterizationCmd(1));
    joystick.x().onTrue(arm.getShoulderKsCharacterizationCmd(2));
    joystick.y().onTrue(arm.getHomeCmd());
  }

  public Command getInitializationCommand() {
    return Commands.sequence(
            // swerve
            swerve.setCustomMaxTiltAccelScaleCommand(() -> arm.getCOGHeightPercent()))
        // swerve.registerBetterPoseCommandSupplier(() -> odometry.getEstimatedPose()),

        .withName("### Robot Initialization ...")
        .ignoringDisable(true);
  }

  public Command getAutonomousCommand() {
    return Commands.none().withName("### Robot Autonomous ...");
  }

  private Command joystickRumbleCommand(double seconds) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds)
        .withName("[Joystick] Rumble " + Math.round(seconds * 10) / 10.0 + "s");
  }
}
