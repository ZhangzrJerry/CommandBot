package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.swerve.Swerve;

public class SuperStructure {
  Swerve swerve;
  Intake intake;
  Arm arm;

  public SuperStructure(Swerve swerve, Intake intake, Arm arm) {
    this.swerve = swerve;
    this.intake = intake;
    this.arm = arm;
  }

  public Command forcedIdleCmd() {
    return Commands.runOnce(
            () -> {
              intake.setGoal(IntakeGoal.IIDLE);
              arm.setGoal(ArmGoal.IDLE);
              intake.setGoal(IntakeGoal.IDLE);
            })
        .withName("$ SuperStructure/Forced Idle");
  }

  public Command algaeIntakeCollectCmd() {
    return Commands.runOnce(
            () -> {
              arm.setGoal(ArmGoal.ALGAE_GROUND_PICK);
              intake.setGoal(IntakeGoal.COLLECT);
            })
        .withName("$ SuperStructure/Algae Intake Collect");
  }

  public Command algaeIntakeEjectCmd() {
    return Commands.runOnce(
            () -> {
              arm.setGoal(ArmGoal.ALGAE_GROUND_PICK);
              intake.setGoal(IntakeGoal.EJECT);
            })
        .withName("$ SuperStructure/Algae Intake Eject");
  }
}
