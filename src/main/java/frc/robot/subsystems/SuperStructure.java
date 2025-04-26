package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class SuperStructure {
  Swerve swerve;
  Intake intake;
  Arm arm;
  Climber climber;
  Endeffector endeffector;

  public SuperStructure(
      Swerve swerve, Intake intake, Arm arm, Climber climber, Endeffector endeffector) {
    this.swerve = swerve;
    this.intake = intake;
    this.arm = arm;
    this.climber = climber;
    this.endeffector = endeffector;
  }

  public Command forcedIdleCmd() {
    return Commands.runOnce(
            () -> {
              intake.setGoal(IntakeGoal.IIDLE); // it's not a typo, it's intentional
              arm.setGoal(ArmGoal.IDLE);
              intake.setGoal(IntakeGoal.IDLE);
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
              endeffector.setCoralGoal(CoralEndEffectorGoal.IDLE);
            })
        .withName("Super/Forced Idle");
  }

  public Command algaeIntakePickCmd() {
    return Commands.startEnd(
            () -> {
              arm.setGoal(ArmGoal.ALGAE_GROUND_PICK);
              intake.setGoal(IntakeGoal.COLLECT);
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
            },
            () -> {
              arm.setGoal(ArmGoal.IDLE);
              intake.setGoal(IntakeGoal.IDLE);
              endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
            })
        .withName("Super/Algae Collect");
  }

  public Command algaeReefPickCmd(boolean highPick) {
    String name = "Super/Algae Reef Pick";
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      if (highPick) {
                        arm.setGoal(ArmGoal.ALGAE_HIGH_PICK);
                        endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
                      } else {
                        arm.setGoal(ArmGoal.ALGAE_LOW_PICK);
                        endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.COLLECT);
                      }
                    })
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
              } else if (arm.getGoal().equals(ArmGoal.ALGAE_NET_SCORE)) {
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
                    })
                .withName(name + "/Set Score Pose"),
            Commands.waitUntil(() -> arm.stopAtGoal()).withName(name + "/Wait For Pose"),
            Commands.runOnce(
                    () -> {
                      endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.SCORE);
                    })
                .withName(name + "/Set Score Goal"),
            Commands.waitUntil(() -> !endeffector.hasAlgaeEndeffectorStoraged())
                .withName(name + "/Wait For Score"),
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.IDLE);
                      endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
                    })
                .withName(name + "/Set Idle Goal"))
        .withName(name);
  }

  public Command algaeProcessorScoreCmd() {
    String name = "Super/Algae Processor Score";
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.ALGAE_PROCESSOR_SCORE);
                    })
                .withName(name + "/Set Score Pose"),
            Commands.waitUntil(() -> arm.stopAtGoal()).withName(name + "/Wait For Pose"),
            Commands.runOnce(
                    () -> {
                      endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.EJECT);
                    })
                .withName(name + "/Set Score Goal"),
            Commands.waitUntil(() -> !endeffector.hasAlgaeEndeffectorStoraged())
                .withName(name + "/Wait For Score"),
            Commands.runOnce(
                    () -> {
                      arm.setGoal(ArmGoal.IDLE);
                      endeffector.setAlgaeGoal(AlgaeEndEffectorGoal.IDLE);
                    })
                .withName(name + "/Set Idle Goal"))
        .withName(name);
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
