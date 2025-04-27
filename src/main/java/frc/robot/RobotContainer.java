package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefScape.GamePiece.Type;
import frc.robot.interfaces.services.PoseService;
import frc.robot.services.GamePieceVisualize;
import frc.robot.services.NodeSelector;
import frc.robot.services.Odometry;
import frc.robot.services.ServiceManager;
import frc.robot.services.TransformTree;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.endeffector.Endeffector;
import frc.robot.subsystems.endeffector.Endeffector.AlgaeEndEffectorGoal;
import frc.robot.subsystems.endeffector.Endeffector.CoralEndEffectorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.AlongWaypointsController;
import frc.robot.subsystems.swerve.controller.TeleopHeadlessController;
import frc.robot.subsystems.vision.AtagVision;
import java.util.Map;
import java.util.Map.Entry;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // ==== command scheduler ====
  CommandScheduler commandScheduler = CommandScheduler.getInstance();

  // ==== service manager ====
  ServiceManager serviceManager = ServiceManager.getInstance();

  // ==== super structure ====
  SuperStructure superStructure;

  // ==== physical subsystems ====
  private final Swerve swerve;
  private final Arm arm;
  private final Intake intake;
  private final Endeffector endeffector;
  private final Climber climber;
  private final AtagVision vision;

  // ==== virtual services ====
  PoseService odometry;
  TransformTree transformTree;
  NodeSelector nodeSelector;
  GamePieceVisualize algaeVisualizer; // -> rely on transform tree
  GamePieceVisualize coralVisualizer; // -> rely on transform tree

  // ==== physical services ====
  private final CommandXboxController joystick =
      new CommandXboxController(Constants.Ports.Joystick.DRIVER);
  private BOTTON_STATE leftBumper = BOTTON_STATE.IDLE;
  private BOTTON_STATE rightBumper = BOTTON_STATE.IDLE;
  private BOTTON_STATE bottonY = BOTTON_STATE.IDLE;

  public RobotContainer() {
    System.out.println("\n>      [0/5] RobotContainer Init ...");

    // ===== instantiate services =====
    transformTree = new TransformTree();
    nodeSelector = new NodeSelector();
    odometry = new Odometry();
    serviceManager.registerService(transformTree, 50);
    serviceManager.registerService(nodeSelector, 20);
    serviceManager.registerService(odometry, 0);

    if (Robot.isReal()) {
      algaeVisualizer = new GamePieceVisualize("Algae Visualizer", new Pose3d[0], new Pose3d[0]);
      coralVisualizer = new GamePieceVisualize("Coral Visualizer", new Pose3d[0], new Pose3d[0]);

      // register services
    } else if (Robot.isSimulation()) {
      algaeVisualizer =
          new GamePieceVisualize(
              "Algae Visualizer",
              ReefScape.GamePiece.Algae.SCORABLE_POSES,
              ReefScape.GamePiece.Algae.PICKABLE_POSES);
      coralVisualizer =
          new GamePieceVisualize(
              "Coral Visualizer",
              ReefScape.GamePiece.Coral.SCORABLE_POSES,
              ReefScape.GamePiece.Coral.PICKABLE_POSES);

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
    superStructure =
        new SuperStructure(swerve, intake, arm, climber, endeffector, nodeSelector, odometry);
    if (Robot.isSimulation()) {
      superStructure
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

    joystick
        .povDown()
        .whileTrue(
            swerve.registerControllerCmd(
                new AlongWaypointsController(
                    () -> odometry.getCurrentPose(),
                    ReefScape.Field.NODE_CONNECTION_MATRIX.getShortestPath(
                        odometry.getCurrentPose(),
                        ReefScape.Field.Reef.getScorePoseBySelection(Type.ALGAE, "AB")))));

    climber.setDefaultCommand(climber.registerTeleopPullCmd(joystick.povDown()));
    new Trigger(
            () ->
                endeffector.hasAlgaeEndeffectorStoraged()
                    || endeffector.hasCoralEndeffectorStoraged())
        .onTrue(joystickRumbleCmd(0.3))
        .onFalse(joystickRumbleCmd(0.2));

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
        .onTrue(superStructure.setClimbingModeCmd().alongWith(joystickRumbleCmd(0.3)));

    // ===== teleop commands =====

    // ##### LT: coral magic eject #####
    joystick
        .leftTrigger(0.3)
        .and(() -> !climber.isClimbing())
        .whileTrue(superStructure.coralMagicEjectCmd());

    // ##### RT: algae magic eject #####
    joystick
        .rightTrigger(0.3)
        .and(() -> !climber.isClimbing())
        .whileTrue(superStructure.algaeMagicEjectCmd());

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
            superStructure
                .coralStationPickCmd(true)
                .alongWith(Commands.runOnce(() -> leftBumper = BOTTON_STATE.FUNC1)));

    joystick
        .leftBumper()
        .and(() -> !climber.isClimbing())
        .and(() -> endeffector.hasCoralEndeffectorStoraged())
        .and(() -> leftBumper != BOTTON_STATE.FUNC1)
        .whileTrue(
            superStructure
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
        .and(() -> true || rightBumper != BOTTON_STATE.FUNC2)
        .whileTrue(
            superStructure
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
                .and(() -> true || rightBumper != BOTTON_STATE.FUNC1)
                .whileTrue(
                    superStructure
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
            superStructure
                .algaeReefPickCmd()
                .alongWith(Commands.runOnce(() -> bottonY = BOTTON_STATE.FUNC2)));

    joystick
        .y()
        .and(() -> !climber.isClimbing())
        .and((() -> endeffector.hasAlgaeEndeffectorStoraged()))
        .and(() -> bottonY != BOTTON_STATE.FUNC2)
        .whileTrue(
            superStructure
                .algaeNetScoreCmd()
                .alongWith(Commands.runOnce(() -> bottonY = BOTTON_STATE.FUNC1)));

    // ##### X: algae ground intake #####
    joystick.x().and(() -> !climber.isClimbing()).whileTrue(superStructure.algaeIntakePickCmd());

    // ##### algae ground pick / processor score #####
    joystick
        .b()
        .and(() -> !climber.isClimbing())
        .whileTrue(superStructure.algaeProcessorScoreCmd());

    // ##### arm idle / arm home #####
    joystick.a().and(() -> !climber.isClimbing()).onTrue(superStructure.forcedIdleCmd());

    joystick
        .a()
        .and(() -> !climber.isClimbing())
        .debounce(0.3)
        .onTrue(arm.getHomeCmd().alongWith(joystickRumbleCmd(0.3)));

    // ##### home gyro #####
    joystick.start().onTrue(superStructure.resetHeadingCmd());
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
    swerve.registerTransform(transformTree);
    arm.registerTransform(transformTree);
    intake.registerTransform(transformTree);
    endeffector.registerTransform(transformTree);
    climber.registerTransform(transformTree);

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

  public enum BOTTON_STATE {
    IDLE,
    FUNC1,
    FUNC2
  }
}
