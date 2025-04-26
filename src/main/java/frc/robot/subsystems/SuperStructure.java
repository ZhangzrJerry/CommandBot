package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmGoal;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberGoal;
import frc.robot.subsystems.endeffector.Endeffector;
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
              intake.setGoal(IntakeGoal.IIDLE);
              arm.setGoal(ArmGoal.IDLE);
              intake.setGoal(IntakeGoal.IDLE);
            })
        .withName("Super/Forced Idle");
  }

  public Command algaeIntakeCollectCmd() {
    return Commands.runOnce(
            () -> {
              arm.setGoal(ArmGoal.ALGAE_GROUND_PICK);
              intake.setGoal(IntakeGoal.COLLECT);
            })
        .withName("Super/Algae Intake Collect");
  }

  public Command algaeIntakeEjectCmd() {
    return Commands.runOnce(
            () -> {
              arm.setGoal(ArmGoal.ALGAE_GROUND_PICK);
              intake.setGoal(IntakeGoal.EJECT);
            })
        .withName("Super/Algae Intake Eject");
  }

  public Command setClimbingCmd() {
    return Commands.runOnce(
            () -> {
              climber.setGoal(ClimberGoal.READY);
              arm.setGoal(ArmGoal.CLIMB);
            })
        .withName("Super/Set Climbing");
  }
}
