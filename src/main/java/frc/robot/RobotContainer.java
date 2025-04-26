package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.services.GamePieceVisualize;
import frc.robot.services.NodeSelector;
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
import frc.robot.subsystems.swerve.controller.TeleopHeaderController;
import frc.robot.subsystems.swerve.controller.TeleopHeadlessController;
import frc.robot.subsystems.vision.AtagVision;

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
    TransformTree transformTree;
    NodeSelector nodeSelector;
    GamePieceVisualize algaeVisualizer; // -> rely on transform tree
    GamePieceVisualize coralVisualizer; // -> rely on transform tree

    private final CommandXboxController joystick = new CommandXboxController(Constants.Ports.Joystick.DRIVER);

    public RobotContainer() {
        System.out.println("\n>      [0/5] RobotContainer Init ...");

        // ===== instantiate services =====
        transformTree = new TransformTree();
        nodeSelector = new NodeSelector();
        serviceManager.registerService(transformTree, 20);
        serviceManager.registerService(nodeSelector, 0);

        if (Robot.isReal()) {
            algaeVisualizer = new GamePieceVisualize("Algae Visualizer", new Pose3d[0], new Pose3d[0]);
            coralVisualizer = new GamePieceVisualize("Coral Visualizer", new Pose3d[0], new Pose3d[0]);

            // register services
        } else if (Robot.isSimulation()) {
            algaeVisualizer = new GamePieceVisualize(
                    "Algae Visualizer",
                    ReefScape.GamePiece.Algae.SCORABLE_POSES,
                    ReefScape.GamePiece.Algae.PICKABLE_POSES);
            coralVisualizer = new GamePieceVisualize(
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
            endeffector = Endeffector.createSim(
                    () -> algaeVisualizer.isHasGamePiece(), () -> coralVisualizer.isHasGamePiece());
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
                        new TeleopHeadlessController(
                                () -> -joystick.getLeftY(),
                                () -> -joystick.getLeftX(),
                                () -> -joystick.getRightX(),
                                () -> swerve.getPose().getRotation())));
        climber.setDefaultCommand(climber.registerTeleopPullCmd(joystick.povDown()));
        new Trigger(
                () -> endeffector.hasAlgaeEndeffectorStoraged()
                        || endeffector.hasCoralEndeffectorStoraged())
                .onTrue(joystickRumbleCmd(0.3));

        // ===== bind mode commands =====
        joystick
                .leftTrigger()
                .and(joystick.rightTrigger())
                .debounce(0.3)
                .whileTrue(
                        Commands.runOnce(
                                () -> nodeSelector.setIgnoreArmMoveCondition(
                                        !nodeSelector.getIgnoreArmMoveCondition()))
                                .alongWith(joystickRumbleCmd(0.3))
                                .ignoringDisable(true));
        joystick
                .back()
                .debounce(0.3)
                .onTrue(superStructure.setClimbingModeCmd().alongWith(joystickRumbleCmd(0.3)));

        // ===== bind teleop commands =====

        // ##### climbing mode #####

        // ##### coral magic eject #####
        joystick
                .leftTrigger(0.3)
                .and(() -> !climber.isClimbing())
                .whileTrue(superStructure.coralMagicEjectCmd());

        // ##### algae magic eject #####
        joystick
                .rightTrigger(0.3)
                .and(() -> !climber.isClimbing())
                .whileTrue(superStructure.algaeMagicEjectCmd());

        // ##### coral station pick #####
        joystick
                .leftBumper()
                .and(() -> !climber.isClimbing())
                .onTrue(superStructure.coralStationPickCmd().withTimeout(2.0));

        // ##### algae reef pick / net score #####
        joystick
                .rightBumper()
                .and(() -> !climber.isClimbing())
                .and(() -> !endeffector.hasAlgaeEndeffectorStoraged())
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    Boolean isAlgaeHighPick = ReefScape.PoseUtils.isAlgaeHighPick(swerve.getPose());
                                    SmartDashboard.putBoolean("isAlgaeHighPick", isAlgaeHighPick);
                                    superStructure.algaeReefPickCmd(isAlgaeHighPick);
                                }));
        joystick
                .rightBumper()
                .and(() -> !climber.isClimbing())
                .and(() -> endeffector.hasAlgaeEndeffectorStoraged())
                .onTrue(superStructure.algaeNetScoreCmd());

        // ##### coral reef score L4/L3/L2 #####
        joystick.y().and(joystick.povUpLeft()).onTrue(superStructure.coralReefScoreCmd(4));
        joystick.y().and(joystick.povUpRight()).onTrue(superStructure.coralReefScoreCmd(4));
        joystick.y().and(joystick.povLeft()).onTrue(superStructure.coralReefScoreCmd(3));
        joystick.y().and(joystick.povRight()).onTrue(superStructure.coralReefScoreCmd(3));
        joystick.y().and(joystick.povDownLeft()).onTrue(superStructure.coralReefScoreCmd(2));
        joystick.y().and(joystick.povDownRight()).onTrue(superStructure.coralReefScoreCmd(2));

        // ##### coral reef score L1 #####
        joystick
                .x()
                .onTrue(
                        superStructure
                                .coralReefScoreCmd(1)
                                .alongWith(
                                        swerve.registerControllerCmd(
                                                new TeleopHeaderController(
                                                        () -> joystick.getLeftY(),
                                                        () -> joystick.getLeftX(),
                                                        () -> -joystick.getRightX()))));

        // ##### arm idle / arm home #####
        joystick.a().and(() -> !climber.isClimbing()).onTrue(superStructure.forcedIdleCmd());
        joystick
                .a()
                .and(() -> !climber.isClimbing())
                .debounce(0.3)
                .onTrue(arm.getHomeCmd().alongWith(joystickRumbleCmd(0.3)));

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

        joystick
                .povUp()
                .whileTrue(
                        Commands.runOnce(
                                () -> SmartDashboard.putBoolean(
                                        "asdf", ReefScape.PoseUtils.isAlgaeHighPick(swerve.getPose()))));
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
                () -> new Pose3d(swerve.getPose())
                        .plus(
                                transformTree.getComponentTransform(
                                        Constants.Ascope.Component.ALGAE_END_EFFECTOR)));
        algaeVisualizer.setScoreMechanismPoseSupplier(
                () -> new Pose3d(swerve.getPose())
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
                () -> new Pose3d(swerve.getPose())
                        .plus(
                                transformTree.getComponentTransform(
                                        Constants.Ascope.Component.CORAL_END_EFFECTOR)));
        coralVisualizer.setScoreMechanismPoseSupplier(
                () -> new Pose3d(swerve.getPose())
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
}
