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
import frc.robot.subsystems.intake.Intake;
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
  private final Intake intake;
  private final AtagVision vision;

  // virtual services
  VisualizeService visualizer;

  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);

  public RobotContainer() {
    System.out.println("\n>      [0/5] RobotContainer Init ...");

    // ===== instantiate services =====
    visualizer = new VisualizeService();
    serviceManager.registerService(visualizer);
    System.out.println("=>     [1/5] Service Register Done");

    // ===== instantiate subsystems =====
    if (Robot.isReal()) {
      swerve = Swerve.createReal();
      arm = Arm.createReal();
      intake = Intake.createReal();
      vision = AtagVision.createReal();
    } else if (Robot.isSimulation()) {
      swerve = Swerve.createSim();
      arm = Arm.createSim();
      intake = Intake.createSim();
      vision = AtagVision.createSim(() -> swerve.getPose());
    } else {
      swerve = Swerve.createIO();
      arm = Arm.createIO();
      intake = Intake.createIO();
      vision = AtagVision.createIO();
    }
    superStructure = new SuperStructure(swerve, intake, arm);
    System.out.println("==>    [2/5] Subsystem Instantiate Done");

    // ======= configure signal subscribers =======
    configureSignalBinding();
    serviceManager.initAll();
    System.out.println("===>   [3/5] Signal Binding Done");

    // ====== configure button bindings ======
    if (commandScheduler.getActiveButtonLoop() != null) {
      commandScheduler.getActiveButtonLoop().clear();
    }
    configureCommandBinding();
    System.out.println("====>  [4/5] Command Binding Done");
    System.out.println("=====> [5/5] RobotContainer Init Done\n");
  }

  void configureCommandBinding() {
    // ===== bind default commands =====
    swerve.setDefaultCommand(
        swerve.registerControllerCmd(
            new TeleopHeaderController(
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX())));

    // ===== bind custom commands =====
    joystick.a().onTrue(superStructure.algaeIntakeCollectCmd());
    joystick.b().onTrue(superStructure.algaeIntakeEjectCmd());
    joystick.x().onTrue(arm.getShoulderKsCharacterizationCmd(2));
    joystick.y().onTrue(arm.getHomeCmd());
    joystick.b().onTrue(superStructure.forcedIdleCmd());
  }

  void configureSignalBinding() {
    // ===== swerve accel limit signal =====
    swerve.setCustomMaxTiltAccelScale(() -> arm.getCOGHeightPercent());

    // ===== intake dodge signal =====
    intake.setDodgeSignalSupplier(() -> arm.needGroundIntakeDodge());

    // ===== visualize service =====
    swerve.registerVisualize(visualizer);
    arm.registerVisualize(visualizer);
    intake.registerVisualize(visualizer);
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
