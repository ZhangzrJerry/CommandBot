package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.services.ServiceManager;
import frc.robot.services.Visualize;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.endeffector.Endeffector;
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
  private final Endeffector endeffector;
  private final Climber climber;

  private final AtagVision vision;

  // virtual services
  Visualize visualizer;

  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);

  public RobotContainer() {
    System.out.println("\n>      [0/5] RobotContainer Init ...");

    // ===== instantiate services =====
    visualizer = new Visualize();
    serviceManager.registerService(visualizer);
    System.out.println("=>     [1/5] Service Register Done");

    // ===== instantiate subsystems =====
    if (Robot.isReal()) {
      swerve = Swerve.createReal();
      arm = Arm.createReal();
      intake = Intake.createReal();
      endeffector = Endeffector.createReal();
      climber = Climber.createReal();
      vision = AtagVision.createReal();
    } else if (Robot.isSimulation()) {
      swerve = Swerve.createSim();
      arm = Arm.createSim();
      intake = Intake.createSim();
      climber = Climber.createSim();
      endeffector = Endeffector.createSim(() -> false, () -> false);
      vision = AtagVision.createSim(() -> swerve.getPose());
    } else {
      swerve = Swerve.createIO();
      arm = Arm.createIO();
      intake = Intake.createIO();
      vision = AtagVision.createIO();
      endeffector = Endeffector.createIO();
      climber = Climber.createIO();
    }
    superStructure = new SuperStructure(swerve, intake, arm, climber, endeffector);
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
    climber.setDefaultCommand(climber.registerTeleopPullCmd(joystick.povDown()));
    new Trigger(
            () ->
                endeffector.hasAlgaeEndeffectorStoraged()
                    || endeffector.hasCoralEndeffectorStoraged())
        .onTrue(joystickRumbleCmd(0.3));

    // ===== bind custom commands =====

    // ##### climbing mode #####
    joystick.back().debounce(0.3).onTrue(superStructure.setClimbingModeCmd());
    joystick.back().debounce(0.3).onTrue(joystickRumbleCmd(0.3));

    // ##### arm idle / arm home #####
    joystick.a().and(() -> !climber.isClimbing()).onTrue(superStructure.forcedIdleCmd());
    joystick.a().and(() -> !climber.isClimbing()).debounce(0.3).onTrue(arm.getHomeCmd());
    joystick.a().and(() -> !climber.isClimbing()).debounce(0.3).onTrue(joystickRumbleCmd(0.3));

    // ##### algae ground pick / processor score #####
    joystick
        .b()
        .and(() -> !climber.isClimbing())
        .and(() -> !endeffector.hasAlgaeEndeffectorStoraged())
        .whileTrue(superStructure.algaeIntakePickCmd());
    joystick
        .b()
        .and(() -> !climber.isClimbing())
        .and(() -> endeffector.hasAlgaeEndeffectorStoraged())
        .onTrue(superStructure.algaeProcessorScoreCmd());

    // ##### algae reef pick / net score #####
    joystick
        .rightBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> !endeffector.hasAlgaeEndeffectorStoraged())
        .onTrue(superStructure.algaeReefPickCmd(true));
    joystick
        .rightBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> endeffector.hasAlgaeEndeffectorStoraged())
        .onTrue(superStructure.algaeNetScoreCmd());

    // ##### algae magic eject #####
    joystick
        .rightTrigger(0.3)
        .and(() -> !climber.isClimbing())
        .whileTrue(superStructure.algaeMagicEjectCmd());

    // ##### coral station pick / reef score #####
    joystick
        .leftBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> !endeffector.hasCoralEndeffectorStoraged())
        .onTrue(superStructure.coralStationPickCmd().withTimeout(2.0));
    joystick
        .leftBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> endeffector.hasCoralEndeffectorStoraged())
        .onTrue(superStructure.coralReefScoreCmd(4));
  }

  void configureSignalBinding() {
    // ===== swerve accel limit signal =====
    swerve.setCustomMaxTiltAccelScale(() -> 1.0 - arm.getCOGHeightPercent());

    // ===== intake dodge signal =====
    intake.setDodgeSignalSupplier(() -> arm.needGroundIntakeDodge());

    // ===== endeffector substitute signal =====
    endeffector.setAlgaeSignalSupplier(joystick.povLeft());
    endeffector.setCoralSignalSupplier(joystick.povRight());

    // ===== visualize service =====
    swerve.registerVisualize(visualizer);
    arm.registerVisualize(visualizer);
    intake.registerVisualize(visualizer);
    endeffector.registerVisualize(visualizer);
    climber.registerVisualize(visualizer);
  }

  public Command getAutoCmd() {
    return Commands.none().withName("### Robot Autonomous ...");
  }

  private Command joystickRumbleCmd(double seconds) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds)
        .withName("Joystick/Rumble " + Math.round(seconds * 10) / 10.0 + "s");
  }
}
