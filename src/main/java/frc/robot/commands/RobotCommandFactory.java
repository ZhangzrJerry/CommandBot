package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefScape;
import frc.robot.interfaces.services.PoseService;
import frc.robot.services.NodeSelector;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmGoal;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberGoal;
import frc.robot.subsystems.endeffector.Endeffector;
import frc.robot.subsystems.endeffector.Endeffector.AlgaeEndEffectorGoal;
import frc.robot.subsystems.endeffector.Endeffector.CoralEndEffectorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.AutoAlignController;
import frc.robot.subsystems.swerve.controller.AutoAlignController.AlignType;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import java.util.function.BooleanSupplier;

public class RobotCommandFactory {
  Swerve swerve;
  Intake intake;
  Arm arm;
  Climber climber;
  Endeffector endeffector;

  NodeSelector nodeSelector;
  PoseService odometry;
  BooleanSupplier shouldArmLift =
      () -> swerve.atToleranceGoal() || nodeSelector.getIgnoreArmMoveCondition();

  public RobotCommandFactory(
      Swerve swerve,
      Intake intake,
      Arm arm,
      Climber climber,
      Endeffector endeffector,
      NodeSelector nodeSelector,
      PoseService odometry) {
    // subsystems
    this.swerve = swerve;
    this.intake = intake;
    this.arm = arm;
    this.climber = climber;
    this.endeffector = endeffector;

    // services
    this.nodeSelector = nodeSelector;
    this.odometry = odometry;
  }

  public Command forcedIdleCmd() {
    return Commands.runOnce(
            () -> {
              intake.setGoal(IntakeGoal.IIDLE); // it's not a typo, it's intentional
              arm.setGoal(ArmGoal.IDLE);
              intake.setGoal(IntakeGoal.IDLE);
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
              endeffector.setCoralGoal(CoralEndEffectorGoal.IDLE);
              swerve.getDefaultCommand();
            },
            swerve)
        .withName("Super/Forced Idle");
  }

  public Command algaeIntakePickCmd() {
    return Commands.run(
            () -> {
              arm.setGoal(ArmGoal.ALGAE_GROUND_PICK);
              intake.setGoal(IntakeGoal.COLLECT);
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
            })
        .finallyDo(
            () -> {
              arm.setGoal(ArmGoal.IDLE);
              intake.setGoal(IntakeGoal.IDLE);
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
            })
        .withName("Super/Algae Collect");
  }

  public Command algaeReefPickCmd() {
    String name = "Super/Algae Reef Pick";
    return Commands.parallel(
            swerve.registerControllerCmd(
                new AutoAlignController(
                    AlignType.REEF_ALGAE,
                    () -> ReefScape.Field.Reef.getAlgaeScoredPose(odometry.getCurrentPose()),
                    () -> odometry.getCurrentPose())),
            Commands.sequence(
                Commands.waitUntil(() -> swerve.atToleranceGoal(3)),
                Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.IDLE);
                    }),
                Commands.waitUntil(shouldArmLift),
                Commands.runOnce(
                        () -> {
                          endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
                          arm.setGoal(
                              ReefScape.PoseUtils.isAlgaeHighPick(odometry.getCurrentPose())
                                  ? ArmGoal.ALGAE_HIGH_PICK
                                  : ArmGoal.ALGAE_LOW_PICK);
                        },
                        arm)
                    .withName(name + "/Set Pick Goal")))
        .withName(name);
  }

  public Command algaeMagicEjectCmd() {
    return Commands.startEnd(
            () -> {
              if (arm.getGoal().equals(ArmGoal.ALGAE_GROUND_PICK)) {
                intake.setGoal(IntakeGoal.EJECT);
                endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.EJECT);
              } else if (arm.getGoal().equals(ArmGoal.ALGAE_NET_SCORE)
                  || arm.getGoal().equals(ArmGoal.ALGAE_PROCESSOR_SCORE)) {
                endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.SCORE);
              } else {
                endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.EJECT);
              }
            },
            () -> endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE))
        .withName("Super/Algae Magic Eject");
  }

  public Command algaeNetScoreCmd() {
    String name = "Super/Algae Net Score";
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new AutoAlignController(
                    AlignType.NET,
                    () -> ReefScape.Field.Barge.getAlgaeScoredPose(odometry.getCurrentPose()),
                    () -> odometry.getCurrentPose())),
            () -> nodeSelector.getIgnoreArmMoveCondition()),
        Commands.sequence(
            Commands.waitUntil(() -> swerve.atToleranceGoal(3)),
            Commands.runOnce(
                () -> {
                  arm.setGoal(ArmGoal.IDLE);
                }),
            Commands.waitUntil(shouldArmLift),
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.ALGAE_NET_SCORE);
                    },
                    arm)
                .withName(name + "/Set Score Pose")));
  }

  public Command algaeNetScoreCmd(double dist) {
    String name = "Super/Algae Net Score";
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new AutoAlignController(
                    AlignType.NET,
                    () -> ReefScape.Field.Barge.getAlgaeScoredPose(dist),
                    () -> odometry.getCurrentPose())),
            () -> nodeSelector.getIgnoreArmMoveCondition()),
        Commands.sequence(
            Commands.waitUntil(() -> swerve.atToleranceGoal(3)),
            Commands.runOnce(
                () -> {
                  arm.setGoal(ArmGoal.IDLE);
                }),
            Commands.waitUntil(shouldArmLift),
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.ALGAE_NET_SCORE);
                    },
                    arm)
                .withName(name + "/Set Score Pose")));
  }

  public Command algaeProcessorScoreCmd() {
    String name = "Super/Algae Processor Score";
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new AutoAlignController(
                    AlignType.PROCESSOR,
                    () ->
                        AllianceFlipUtil.isRobotInBlueSide(odometry.getCurrentPose())
                            ? ReefScape.Field.Processor.SCORE_POSE
                            : AllianceFlipUtil.flipPose(ReefScape.Field.Processor.SCORE_POSE),
                    () -> odometry.getCurrentPose())),
            () -> nodeSelector.getIgnoreArmMoveCondition()),
        Commands.sequence(
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.ALGAE_PROCESSOR_SCORE);
                    },
                    arm)
                .withName(name + "/Set Score Pose")));
  }

  public Command coralStationPickCmd(boolean isLeft) {
    String name = "Super/Coral Station Pick";
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new AutoAlignController(
                    isLeft ? AlignType.CORAL_STATION_LEFT : AlignType.CORAL_STATION_RIGHT,
                    () ->
                        AllianceFlipUtil.isRobotInBlueSide(odometry.getCurrentPose())
                            ? (isLeft
                                ? ReefScape.Field.CoralStation.LEFT_CENTER_COLLECT_POSE
                                : ReefScape.Field.CoralStation.RIGHT_CENTER_COLLECT_POSE)
                            : (isLeft
                                ? AllianceFlipUtil.flipPose(
                                    ReefScape.Field.CoralStation.LEFT_CENTER_COLLECT_POSE)
                                : AllianceFlipUtil.flipPose(
                                    ReefScape.Field.CoralStation.RIGHT_CENTER_COLLECT_POSE)),
                    () -> odometry.getCurrentPose())),
            () -> nodeSelector.getIgnoreArmMoveCondition()),
        Commands.sequence(
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.CORAL_STATION_COLLECT);
                      endeffector.setCoralGoal(CoralEndEffectorGoal.COLLECT);
                    },
                    arm)
                .withName(name + "/Set Collect Pose")));
  }

  public Command coralReefScoreCmd(int layer, boolean isLeft) {
    String name = "Super/Coral Reef Score";
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new AutoAlignController(
                    AlignType.REEF_CORAL,
                    () ->
                        layer > 1
                            ? ReefScape.Field.Reef.getCoralScoredPose(
                                odometry.getCurrentPose(), isLeft)
                            : ReefScape.Field.Reef.getAlgaeScoredPose(odometry.getCurrentPose()),
                    () -> odometry.getCurrentPose())),
            () -> nodeSelector.getIgnoreArmMoveCondition()),
        Commands.sequence(
            Commands.waitUntil(() -> swerve.atToleranceGoal(3)),
            Commands.runOnce(
                () -> {
                  arm.setGoal(ArmGoal.IDLE);
                }),
            Commands.waitUntil(shouldArmLift),
            Commands.runOnce(
                    () -> {
                      switch (layer) {
                        case 1:
                          arm.setGoal(ArmGoal.CORAL_L1_SCORE);
                          break;
                        case 2:
                          arm.setGoal(ArmGoal.CORAL_L2_SCORE);
                          break;
                        case 3:
                          arm.setGoal(ArmGoal.CORAL_L3_SCORE);
                          break;
                        case 4:
                          arm.setGoal(ArmGoal.CORAL_L4_SCORE);
                          break;
                      }
                    })
                .withName(name + "/Set Score Pose")));
  }

  public Command coralReefScoreCmd() {
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new AutoAlignController(
                    AlignType.REEF_CORAL,
                    () ->
                        !AllianceFlipUtil.isRobotInBlueSide(odometry.getCurrentPose())
                            ? AllianceFlipUtil.flipPose(
                                ReefScape.Field.Reef.getScorePoseBySelection(
                                    ReefScape.GamePiece.Type.CORAL,
                                    nodeSelector.getSelectedBranch()))
                            : ReefScape.Field.Reef.getScorePoseBySelection(
                                ReefScape.GamePiece.Type.CORAL, nodeSelector.getSelectedBranch()),
                    () -> odometry.getCurrentPose())),
            () -> nodeSelector.getIgnoreArmMoveCondition()),
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  arm.setGoal(ArmGoal.IDLE);
                }),
            Commands.waitUntil(shouldArmLift),
            Commands.runOnce(
                () -> {
                  switch (nodeSelector.getSelectedLevel()) {
                    case "1":
                      arm.setGoal(ArmGoal.CORAL_L1_SCORE);
                      break;
                    case "2":
                      arm.setGoal(ArmGoal.CORAL_L2_SCORE);
                      break;
                    case "3":
                      arm.setGoal(ArmGoal.CORAL_L3_SCORE);
                      break;
                    case "4":
                      arm.setGoal(ArmGoal.CORAL_L4_SCORE);
                      break;
                  }
                })));
  }

  public Command coralMagicEjectCmd() {
    return Commands.startEnd(
            () -> {
              if (arm.getGoal().equals(ArmGoal.CORAL_L1_SCORE)
                  || arm.getGoal().equals(ArmGoal.CORAL_L2_SCORE)
                  || arm.getGoal().equals(ArmGoal.CORAL_L3_SCORE)
                  || arm.getGoal().equals(ArmGoal.CORAL_L4_SCORE)) {
                endeffector.setCoralGoal(CoralEndEffectorGoal.SCORE);
              } else {
                endeffector.setCoralGoal(CoralEndEffectorGoal.EJECT);
              }
            },
            () -> endeffector.setCoralGoal(CoralEndEffectorGoal.IDLE))
        .withName("Super/Coral Magic Eject");
  }

  public Command setClimbingModeCmd() {
    return Commands.runOnce(
            () -> {
              climber.setGoal(ClimberGoal.READY);
              arm.setGoal(ArmGoal.CLIMB);
            })
        .withName("Super/Set Climbing");
  }

  public Command resetPoseCmd(Pose2d pose) {
    return Commands.runOnce(
            () -> {
              odometry.resetPose(pose);
              swerve.resetWheeledPoseCmd(new UncertainPose2d(pose)).schedule();
            })
        .withName("Super/Reset Pose");
  }

  public Command resetHeadingCmd() {
    return Commands.runOnce(
            () -> {
              swerve
                  .resetGyroHeadingCmd(
                      AllianceFlipUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)
                  .schedule();
              odometry.resetHeading(
                  AllianceFlipUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
            },
            swerve)
        .withName("Super/Reset Heading");
  }

  public Command laserNetScoreCmd(double dist) {
    return Commands.sequence(
        algaeNetScoreCmd(dist),
        Commands.waitUntil(() -> swerve.atGoal() && arm.stopAtGoal()),
        Commands.runOnce(
            () -> {
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.SCORE);
            }),
        Commands.waitUntil(() -> !endeffector.hasAlgaeEndeffectorStoraged()),
        Commands.runOnce(
            () -> {
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
            }));
  }

  public Command laserReefScoreCmd(String branch, String layer) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              nodeSelector.setSelected(ReefScape.GamePiece.Type.CORAL, branch, layer);
            }),
        Commands.waitUntil(
            () ->
                nodeSelector.getSelectedLevel().equals(layer)
                    && nodeSelector.getSelectedBranch().equals(branch)),
        coralReefScoreCmd(),
        Commands.waitUntil(() -> swerve.atGoal() && arm.stopAtGoal()),
        Commands.runOnce(
            () -> {
              endeffector.setCoralGoal(CoralEndEffectorGoal.SCORE);
            }),
        Commands.waitUntil(() -> !endeffector.hasCoralEndeffectorStoraged()),
        Commands.runOnce(
            () -> {
              endeffector.setCoralGoal(CoralEndEffectorGoal.IDLE);
            }));
  }

  public Command laserReefCollectCmd(String branch) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              nodeSelector.setSelected(ReefScape.GamePiece.Type.CORAL, branch, "");
            }),
        swerve.registerControllerCmd(
            new AutoAlignController(
                AlignType.REEF_CORAL,
                () ->
                    !AllianceFlipUtil.isRedAlliance()
                        ? ReefScape.Field.Reef.getScorePoseBySelection(
                            ReefScape.GamePiece.Type.ALGAE, branch)
                        : AllianceFlipUtil.flipPose(
                            ReefScape.Field.Reef.getScorePoseBySelection(
                                ReefScape.GamePiece.Type.ALGAE, branch)),
                () -> odometry.getCurrentPose())),
        Commands.waitUntil(() -> swerve.atGoal() && arm.stopAtGoal()),
        Commands.runOnce(
            () -> {
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
            }),
        Commands.waitUntil(() -> endeffector.hasAlgaeEndeffectorStoraged()),
        Commands.runOnce(
            () -> {
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.HOLDING);
            }));
  }

  public Command tempAutoCmd() {
    return Commands.sequence(
        resetPoseCmd(AllianceFlipUtil.applyPose(new Pose2d(7, 4, Rotation2d.k180deg))),
        laserReefScoreCmd("E", "4"),
        coralStationPickCmd(false),
        Commands.waitUntil(() -> endeffector.hasCoralEndeffectorStoraged()),
        laserReefScoreCmd("D", "4"),
        coralStationPickCmd(false),
        Commands.waitUntil(() -> endeffector.hasCoralEndeffectorStoraged()),
        laserReefScoreCmd("C", "4"),
        coralStationPickCmd(false),
        Commands.waitUntil(() -> endeffector.hasCoralEndeffectorStoraged()),
        laserReefScoreCmd("B", "4"),
        coralStationPickCmd(false),
        Commands.waitUntil(() -> endeffector.hasCoralEndeffectorStoraged()),
        laserReefScoreCmd("A", "4"),
        coralStationPickCmd(false),
        Commands.waitUntil(() -> endeffector.hasCoralEndeffectorStoraged()));
  }
}
