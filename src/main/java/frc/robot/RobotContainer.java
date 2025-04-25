package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.services.ServiceManager;
import frc.robot.services.VisualizeService;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.TeleopHeaderController;
import frc.robot.subsystems.vision.AtagVision;

public class RobotContainer {
  //
  CommandScheduler commandScheduler = CommandScheduler.getInstance();
  ServiceManager serviceManager = ServiceManager.getInstance();
  SuperStructure superStructure;

  // physical subsystems
  private final Swerve swerve;
  private final Arm arm;
  private final AtagVision vision;

  // virtual services
  VisualizeService visualize;

  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);

  public RobotContainer() {
    // ===== instantiate services =====
    visualize = new VisualizeService();
    serviceManager.registerService(visualize);
    System.out.println("$$$ Service Register Done");

    // ===== instantiate subsystems =====
    if (Robot.isReal()) {
      swerve = Swerve.createReal();
      arm = Arm.createReal();
      vision = AtagVision.createReal();
    } else if (Robot.isSimulation()) {
      swerve = Swerve.createSim();
      arm = Arm.createSim();
      vision = AtagVision.createSim(() -> swerve.getPose());
    } else {
      swerve = Swerve.createIO();
      arm = Arm.createIO();
      vision = AtagVision.createIO();
    }
    superStructure = new SuperStructure(swerve);
    System.out.println("$$$ Subsystem Instantiate Done");

    // ======= configure signal subscribers =======
    configureSubscribeRequest();
    serviceManager.initAll();
    System.out.println("$$$ Subscribe Request Done");

    // ====== configure button bindings ======
    commandScheduler.getActiveButtonLoop().clear();
    configureButtonBindings();
    System.out.println("$$$ Button Binding Done");
  }

  void configureSubscribeRequest() {
    // ===== visualize service =====
    visualize.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_FL, -1, swerve::getDrivetrainToFlwheelTransform);
    visualize.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_FR, -1, swerve::getDrivetrainToFrwheelTransform);
    visualize.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_BL, -1, swerve::getDrivetrainToBlwheelTransform);
    visualize.registerVisualizeComponent(
        Constants.Ascope.Component.SWERVE_BR, -1, swerve::getDrivetrainToBrwheelTransform);
  }

  void configureButtonBindings() {
    // ===== bind default commands =====
    swerve.setDefaultCommand(
        swerve.registerControllerCommand(
            new TeleopHeaderController(
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX())));

    // ===== bind custom commands =====
    joystick.a().onTrue(arm.getElbowKsCharacterizationCmd(1));
    joystick.x().onTrue(arm.getShoulderKsCharacterizationCmd(2));
    joystick.y().onTrue(arm.getHomeCmd());
  }

  public Command getAutoCmd() {
    return Commands.none().withName("### Robot Autonomous ...");
  }

  private Command joystickRumbleCmd(double seconds) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds)
        .withName("[Joystick] Rumble " + Math.round(seconds * 10) / 10.0 + "s");
  }
}
