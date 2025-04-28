package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotCommandFactory;
import frc.robot.interfaces.services.PoseService;
import frc.robot.services.CommandSelector;
import frc.robot.services.GamePieceVisualize;
import frc.robot.services.NodeSelector;
import frc.robot.services.Odometry;
import frc.robot.services.ServiceManager;
import frc.robot.services.TransformTree;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.endeffector.Endeffector;
import frc.robot.subsystems.endeffector.Endeffector.AlgaeEndEffectorGoal;
import frc.robot.subsystems.endeffector.Endeffector.CoralEndEffectorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.TeleopHeadlessController;
import frc.robot.subsystems.vision.AtagVision;
import java.util.Map;
import java.util.Map.Entry;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // ==== physical subsystems ====
  private final Swerve swerve;
  private final Arm arm;
  private final Intake intake;
  private final Endeffector endeffector;
  private final Climber climber;
  private final AtagVision vision;

  // ==== command scheduler ====
  CommandScheduler commandScheduler = CommandScheduler.getInstance();
  RobotCommandFactory robotCommand;

  // ==== service manager ====
  ServiceManager serviceManager = ServiceManager.getInstance();
  PoseService odometry;
  TransformTree transformTree;
  NodeSelector nodeSelector;
  CommandSelector autoModeSelector;
  GamePieceVisualize algaeVisualizer; // -> rely on transform tree
  GamePieceVisualize coralVisualizer; // -> rely on transform tree

  // ====
  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);
  private BOTTON_STATE leftBumper = BOTTON_STATE.IDLE;
  private BOTTON_STATE rightBumper = BOTTON_STATE.IDLE;
  private BOTTON_STATE bottonY = BOTTON_STATE.IDLE;

  public RobotContainer() {
    System.out.println("\n>      [0/5] RobotContainer Init ...");

    // ===== instantiate services =====
    transformTree = new TransformTree("Transform Tree");
    nodeSelector = new NodeSelector();
    odometry = new Odometry();
    autoModeSelector = new CommandSelector("Auto Mode Selector", "Auto");
    serviceManager.registerService(transformTree, 50);
    serviceManager.registerService(nodeSelector, 20);
    serviceManager.registerService(odometry, 0);
    serviceManager.registerService(autoModeSelector, 20);

    if (Robot.isReal()) {
      algaeVisualizer =
          new GamePieceVisualize("Algae Visualizer", 0, 0, new Pose3d[0], new Pose3d[0]);
      coralVisualizer =
          new GamePieceVisualize("Coral Visualizer", 0, 0, new Pose3d[0], new Pose3d[0]);

      // register services
    } else if (Robot.isSimulation()) {
      algaeVisualizer =
          new GamePieceVisualize(
              "Algae Visualizer",
              0.4,
              1.0,
              ReefScape.GamePiece.Algae.PICKABLE_POSES,
              ReefScape.GamePiece.Algae.SCORABLE_POSES);
      coralVisualizer =
          new GamePieceVisualize(
              "Coral Visualizer",
              0.45,
              0.4,
              ReefScape.GamePiece.Coral.PICKABLE_POSES,
              ReefScape.GamePiece.Coral.SCORABLE_POSES);

      // register services
      serviceManager.registerService(algaeVisualizer);
      serviceManager.registerService(coralVisualizer);
    }

    System.out.println("=>     [1/5] Service Register Done");

    // ===== instantiate subsystems =====
    if (Robot.isReal()) {
      swerve = Swerve.createReal(odometry);
      arm = Arm.createReal();
      intake = Intake.createReal();
      endeffector = Endeffector.createReal();
      climber = Climber.createReal();
      vision = AtagVision.createReal(odometry);
    } else if (Robot.isSimulation()) {
      swerve = Swerve.createSim(odometry);
      arm = Arm.createSim();
      intake = Intake.createSim();
      climber = Climber.createSim();
      endeffector =
          Endeffector.createSim(
              () -> algaeVisualizer.isHasGamePiece(), () -> coralVisualizer.isHasGamePiece());
      vision = AtagVision.createSim(odometry, () -> swerve.getPose());
    } else {
      swerve = Swerve.createIO();
      arm = Arm.createIO();
      intake = Intake.createIO();
      vision = AtagVision.createIO();
      endeffector = Endeffector.createIO();
      climber = Climber.createIO();
    }
    robotCommand =
        new RobotCommandFactory(swerve, intake, arm, climber, endeffector, nodeSelector, odometry);
    if (Robot.isSimulation()) {
      robotCommand
          .resetPoseCmd(
              new Pose2d(ReefScape.Field.LENGTH / 2, ReefScape.Field.WIDTH / 2, Rotation2d.kZero))
          .schedule();
    }
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

    Logger.recordOutput("Nodes/Preset", ReefScape.Field.NODE_CONNECTION_MATRIX.getNodePoses());
  }

  void configureCommandBinding() {
    // ===== default commands =====

    swerve.setDefaultCommand(
        swerve.registerControllerCmd(
            new TeleopHeadlessController(
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX(),
                () -> odometry.getCurrentHeading())));

    climber.setDefaultCommand(climber.registerTeleopPullCmd(joystick.povDown()));

    new Trigger(() -> endeffector.hasAlgaeEndeffectorStoraged())
        .onTrue(joystickRumbleCmd(0.3))
        .onFalse(joystickRumbleCmd(0.2));

    new Trigger(() -> endeffector.hasCoralEndeffectorStoraged())
        .onTrue(joystickRumbleCmd(0.3))
        .onFalse(joystickRumbleCmd(0.2));

    new Trigger(() -> DriverStation.isAutonomous())
        .onTrue(
            Commands.runOnce(
                () -> {
                  // autoModeSelector.pause();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  // autoModeSelector.resume();
                }));

    // ===== mode commands =====
    joystick
        .leftBumper()
        .and(joystick.rightBumper())
        .debounce(0.3)
        .onTrue(
            Commands.runOnce(
                    () ->
                        nodeSelector.setIgnoreArmMoveCondition(
                            !nodeSelector.getIgnoreArmMoveCondition()))
                .alongWith(joystickRumbleCmd(0.3))
                .ignoringDisable(true));

    joystick
        .back()
        .debounce(0.3)
        .onTrue(robotCommand.setClimbingModeCmd().alongWith(joystickRumbleCmd(0.3)));

    // ===== teleop commands =====

    // ##### LT: coral magic eject #####
    joystick
        .leftTrigger(0.3)
        .and(() -> !climber.isClimbing())
        .whileTrue(robotCommand.coralMagicEjectCmd());

    // ##### RT: algae magic eject #####
    joystick
        .rightTrigger(0.3)
        .and(() -> !climber.isClimbing())
        .whileTrue(robotCommand.algaeMagicEjectCmd());

    // ##### LB: left coral station pick / coral reef score with NodeSelector #####
    joystick
        .leftBumper()
        .debounce(0.1)
        .onFalse(Commands.runOnce(() -> leftBumper = BOTTON_STATE.IDLE));

    joystick
        .leftBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> !endeffector.hasCoralEndeffectorStoraged())
        .and(() -> leftBumper != BOTTON_STATE.FUNC2)
        .whileTrue(
            robotCommand
                .coralStationPickCmd(true)
                .alongWith(Commands.runOnce(() -> leftBumper = BOTTON_STATE.FUNC1)));

    joystick
        .leftBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> endeffector.hasCoralEndeffectorStoraged())
        .and(() -> leftBumper != BOTTON_STATE.FUNC1)
        .whileTrue(
            robotCommand
                .coralReefScoreCmd()
                .alongWith(Commands.runOnce(() -> leftBumper = BOTTON_STATE.FUNC2)));

    // ##### RB: right coral station pick / coral reef score with POV #####
    joystick
        .rightBumper()
        .debounce(0.1)
        .onFalse(Commands.runOnce(() -> rightBumper = BOTTON_STATE.IDLE));

    joystick
        .rightBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> !endeffector.hasCoralEndeffectorStoraged())
        .and(() -> rightBumper != BOTTON_STATE.FUNC2)
        .whileTrue(
            robotCommand
                .coralStationPickCmd(false)
                .alongWith(Commands.runOnce(() -> rightBumper = BOTTON_STATE.FUNC1)));

    Map<Trigger, Entry<Integer, Boolean>> coralScoreTriggers =
        Map.ofEntries(
            Map.entry(joystick.povUpLeft(), Map.entry(4, true)),
            Map.entry(joystick.povUpRight(), Map.entry(4, false)),
            Map.entry(joystick.povLeft(), Map.entry(3, true)),
            Map.entry(joystick.povRight(), Map.entry(3, false)),
            Map.entry(joystick.povDownLeft(), Map.entry(2, true)),
            Map.entry(joystick.povDownRight(), Map.entry(2, false)),
            Map.entry(joystick.povDown(), Map.entry(1, true)));

    coralScoreTriggers.forEach(
        (trigger, params) ->
            joystick
                .rightBumper()
                .and(trigger)
                .and(() -> !climber.isClimbing())
                .and(() -> endeffector.hasCoralEndeffectorStoraged())
                .and(() -> rightBumper != BOTTON_STATE.FUNC1)
                .whileTrue(
                    robotCommand
                        .coralReefScoreCmd(params.getKey(), params.getValue())
                        .alongWith(Commands.runOnce(() -> rightBumper = BOTTON_STATE.FUNC2))));

    // ##### Y: algae reef pick / net score #####
    joystick.y().debounce(0.1).onFalse(Commands.runOnce(() -> bottonY = BOTTON_STATE.IDLE));

    joystick
        .y()
        .and(() -> !climber.isClimbing())
        .and(() -> !endeffector.hasAlgaeEndeffectorStoraged())
        .and(() -> bottonY != BOTTON_STATE.FUNC1)
        .whileTrue(
            robotCommand
                .algaeReefPickCmd()
                .alongWith(Commands.runOnce(() -> bottonY = BOTTON_STATE.FUNC2)));

    joystick
        .y()
        .and(() -> !climber.isClimbing())
        .and((() -> endeffector.hasAlgaeEndeffectorStoraged()))
        .and(() -> bottonY != BOTTON_STATE.FUNC2)
        .whileTrue(
            robotCommand
                .algaeNetScoreCmd()
                .alongWith(Commands.runOnce(() -> bottonY = BOTTON_STATE.FUNC1)));

    // ##### X: algae ground intake #####
    joystick.x().and(() -> !climber.isClimbing()).whileTrue(robotCommand.algaeIntakePickCmd());

    // ##### B: algae ground pick / processor score #####
    joystick.b().and(() -> !climber.isClimbing()).whileTrue(robotCommand.algaeProcessorScoreCmd());

    // ##### A: arm idle / arm home #####
    joystick.a().and(() -> !climber.isClimbing()).onTrue(robotCommand.forcedIdleCmd());

    joystick
        .a()
        .and(() -> !climber.isClimbing())
        .debounce(0.3)
        .onTrue(arm.getHomeCmd().alongWith(joystickRumbleCmd(0.3)));

    // ##### home gyro #####
    joystick.start().onTrue(robotCommand.resetHeadingCmd());
  }

  void configureSignalBinding() {
    // ===== swerve accel limit signal =====
    swerve.setCustomMaxTiltAccelScale(() -> 1.0 - arm.getCOGHeightPercent());

    // ===== intake dodge signal =====
    intake.setDodgeSignalSupplier(() -> arm.needGroundIntakeDodge());

    // ===== endeffector substitute signal =====
    endeffector.setAlgaeSignalSupplier(joystick.povLeft());
    endeffector.setCoralSignalSupplier(joystick.povRight());

    // ===== kinematic service =====
    swerve.registerTransform(transformTree);
    arm.registerTransform(transformTree);
    intake.registerTransform(transformTree);
    endeffector.registerTransform(transformTree);
    climber.registerTransform(transformTree);

    // ===== automode selector service =====
    autoModeSelector.addMode("Example Auto", robotCommand.tempAutoCmd());
    autoModeSelector.addMode(
        "Swerve Wheel Radius Characterization", swerve.getWheelRadiusCharacterizationCmd(true));
    autoModeSelector.addMode("Swerve Ks Characterization", swerve.getKsCharacterizationCmd());
    autoModeSelector.addMode(
        "Arm Shoulder Ks Characterization", arm.getShoulderKsCharacterizationCmd(0.01));
    autoModeSelector.addMode(
        "Arm Elbow Ks Characterization", arm.getElbowKsCharacterizationCmd(0.01));

    // ===== algae game piece visualize =====
    algaeVisualizer.setPickMechanismPoseSupplier(
        () ->
            new Pose3d(odometry.getCurrentPose())
                .plus(
                    transformTree.getComponentTransform(
                        Constants.Ascope.Component.ALGAE_END_EFFECTOR)));
    algaeVisualizer.setScoreMechanismPoseSupplier(
        () ->
            new Pose3d(odometry.getCurrentPose())
                .plus(
                    transformTree.getComponentTransform(
                        Constants.Ascope.Component.ALGAE_END_EFFECTOR)));
    algaeVisualizer.setTryPickSupplier(
        () -> endeffector.getAlgaeGoal().equals(AlgaeEndEffectorGoal.COLLECT));
    algaeVisualizer.setTryEjectSupplier(
        () -> endeffector.getAlgaeGoal().equals(AlgaeEndEffectorGoal.EJECT));
    algaeVisualizer.setTryScoreSupplier(
        () -> endeffector.getAlgaeGoal().equals(AlgaeEndEffectorGoal.SCORE));

    // ===== coral game piece visualize =====
    coralVisualizer.setPickMechanismPoseSupplier(
        () ->
            new Pose3d(odometry.getCurrentPose())
                .plus(
                    transformTree.getComponentTransform(
                        Constants.Ascope.Component.CORAL_END_EFFECTOR)));
    coralVisualizer.setScoreMechanismPoseSupplier(
        () ->
            new Pose3d(odometry.getCurrentPose())
                .plus(
                    transformTree.getComponentTransform(
                        Constants.Ascope.Component.CORAL_END_EFFECTOR)));
    coralVisualizer.setTryPickSupplier(
        () -> endeffector.getCoralGoal().equals(CoralEndEffectorGoal.COLLECT));
    coralVisualizer.setTryEjectSupplier(
        () -> endeffector.getCoralGoal().equals(CoralEndEffectorGoal.EJECT));
    coralVisualizer.setTryScoreSupplier(
        () -> endeffector.getCoralGoal().equals(CoralEndEffectorGoal.SCORE));
    coralVisualizer.setHasGamePiece(true);
  }

  public Command getAutoCmd() {
    return autoModeSelector.getCmd();
  }

  private Command joystickRumbleCmd(double seconds) {
    return Commands.startEnd(
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
            () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withTimeout(seconds)
        .withName("Joystick/Rumble " + Math.round(seconds * 10) / 10.0 + "s");
  }

  public enum BOTTON_STATE {
    IDLE,
    FUNC1,
    FUNC2
  }
}
