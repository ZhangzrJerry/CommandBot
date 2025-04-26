package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefScape;
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
import frc.robot.subsystems.swerve.controller.SwerveAlignController;
import frc.robot.subsystems.swerve.controller.SwerveAlignController.AlignType;
import frc.robot.utils.AllianceFlipUtil;
import java.util.function.BooleanSupplier;

public class SuperStructure {
  Swerve swerve;
  Intake intake;
  Arm arm;
  Climber climber;
  Endeffector endeffector;

  NodeSelector nodeSelector;

  BooleanSupplier shouldArmLift =
      () -> swerve.atToleranceGoal() || nodeSelector.getIgnoreArmMoveCondition();

  public SuperStructure(
      Swerve swerve,
      Intake intake,
      Arm arm,
      Climber climber,
      Endeffector endeffector,
      NodeSelector nodeSelector) {
    // subsystems
    this.swerve = swerve;
    this.intake = intake;
    this.arm = arm;
    this.climber = climber;
    this.endeffector = endeffector;

    // services
    this.nodeSelector = nodeSelector;
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

  public Command algaeReefPickCmd(boolean highPick) {
    String name = "Super/Algae Reef Pick";
    SmartDashboard.putBoolean("name", highPick);
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
                      arm.setGoal(highPick ? ArmGoal.ALGAE_HIGH_PICK : ArmGoal.ALGAE_LOW_PICK);
                    },
                    arm)
                .withName(name + "/Set Pick Goal"),
            Commands.waitUntil(() -> endeffector.hasAlgaeEndeffectorStoraged())
                .withName(name + "/Wait For Pick"),
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.IDLE);
                      endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.HOLDING);
                    })
                .withName(name + "/Set Holding Goal"))
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
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.ALGAE_NET_SCORE);
                    },
                    arm)
                .withName(name + "/Set Score Pose"))
        .withName(name);
  }

  public Command algaeProcessorScoreCmd() {
    String name = "Super/Algae Processor Score";
    return Commands.parallel(
        Commands.either(
            Commands.none(),
            swerve.registerControllerCmd(
                new SwerveAlignController(
                    AlignType.PROCESSOR,
                    () ->
                        AllianceFlipUtil.isRobotInBlueSide(swerve.getPose())
                            ? ReefScape.Field.Processor.SCORE_POSE
                            : AllianceFlipUtil.flipPose(ReefScape.Field.Processor.SCORE_POSE),
                    () -> swerve.getPose())),
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
                new SwerveAlignController(
                    isLeft ? AlignType.CORAL_STATION_LEFT : AlignType.CORAL_STATION_RIGHT,
                    () ->
                        AllianceFlipUtil.isRobotInBlueSide(swerve.getPose())
                            ? (isLeft
                                ? ReefScape.Field.CoralStation.LEFT_CENTER_COLLECT_POSE
                                : ReefScape.Field.CoralStation.RIGHT_CENTER_COLLECT_POSE)
                            : (isLeft
                                ? AllianceFlipUtil.flipPose(
                                    ReefScape.Field.CoralStation.LEFT_CENTER_COLLECT_POSE)
                                : AllianceFlipUtil.flipPose(
                                    ReefScape.Field.CoralStation.RIGHT_CENTER_COLLECT_POSE)),
                    () -> swerve.getPose())),
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

  public Command coralReefScoreCmd(int layer) {
    String name = "Super/Coral Reef Score";
    return Commands.sequence(
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
                .withName(name + "/Set Score Pose"))
        .withName(name);
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
}
