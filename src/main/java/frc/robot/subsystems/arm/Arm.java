package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfx;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfxCancoder;
import frc.robot.utils.GainsUtil.PdsGains;
import frc.robot.utils.GainsUtil.PidsgGains;
import frc.robot.utils.logging.AlertUtil;
import frc.robot.utils.logging.LoggedTunableGains;
import frc.robot.utils.logging.LoggedTunableNumber;
import frc.robot.utils.math.EqualsUtil;
import frc.robot.utils.math.UnitConverter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final LoggedTunableNumber transitionElevatorHeightMeter =
      new LoggedTunableNumber("Arm/TransitionElevatorHeightMeter", 0.38);
  private static final LoggedTunableNumber minSafeGroundIntakeDodgeElevatorHeightMeter =
      new LoggedTunableNumber("Arm/MinSafeGroundIntakeDodgeElevatorHeightMeter", 0.65);

  private static final LoggedTunableNumber shoulderToleranceMeter =
      new LoggedTunableNumber("Arm/Shoulder/ToleranceMeter", 0.1);
  private static final LoggedTunableNumber shoulderStopToleranceMeterPerSec =
      new LoggedTunableNumber("Arm/Shoulder/StopToleranceMeterPerSec", 0.1);
  private static final LoggedTunableNumber shoulderStaticCharacterizationVelocityThreshMeterPerSec =
      new LoggedTunableNumber("Arm/Shoulder/StaticCharacterizationVelocityThreshMeterPerSec", 0.01);
  private static final LoggedTunableNumber shoulderHomingCurrentAmp =
      new LoggedTunableNumber("Arm/Shoulder/HomingCurrentAmp", -8.0);
  private static final LoggedTunableNumber shoulderHomingTimeSecs =
      new LoggedTunableNumber("Arm/Shoulder/HomingTimeSecs", 0.6);
  private static final LoggedTunableNumber shoulderHomingVelocityThreshMeterPerSec =
      new LoggedTunableNumber("Arm/Shoulder/HomingVelocityThreshMeterPerSec", 0.05);

  private static final LoggedTunableNumber shoulderCoralL1HeightMeter =
      new LoggedTunableNumber("Arm/Shoulder/Goal/CoralL1ScoreHeightMeter", 0.04);
  private static final LoggedTunableNumber shoulderCoralL2HeightMeter =
      new LoggedTunableNumber("Arm/Shoulder/Goal/CoralL2ScoreHeightMeter", 0.27);
  private static final LoggedTunableNumber shoulderCoralL3HeightMeter =
      new LoggedTunableNumber("Arm/Shoulder/Goal/CoralL3ScoreHeightMeter", 0.67);
  private static final LoggedTunableNumber shoulderCoralL4HeightMeter =
      new LoggedTunableNumber("Arm/Shoulder/Goal/CoralL4ScoreHeightMeter", 1.55);

  private static final LoggedTunableNumber elbowToleranceDegree =
      new LoggedTunableNumber("Arm/Elbow/ToleranceDegree", 12.0);
  private static final LoggedTunableNumber elbowStopToleranceDegreePerSec =
      new LoggedTunableNumber("Arm/Elbow/StopToleranceDegreePerSec", 10.0);
  private static final LoggedTunableNumber elbowStaticCharacterizationVelocityThreshDegreePerSec =
      new LoggedTunableNumber("Arm/Elbow/StaticCharacterizationVelocityThreshDegreePerSec", 10.0);
  private static final LoggedTunableNumber elbowAvoidReefAlgaePositionDegree =
      new LoggedTunableNumber("Arm/Elbow/AvoidReefAlgaePositionDegree", -70.0);

  private static final LoggedTunableGains<PidsgGains> shoulderGains;
  private static final LoggedTunableGains<PdsGains> elbowGains;

  static {
    shoulderGains =
        new LoggedTunableGains<PidsgGains>("Arm/Shoulder/Gains", ArmConfig.SHOULDER_GAINS);
    elbowGains = new LoggedTunableGains<PdsGains>("Arm/Elbow/Gains", ArmConfig.ELBOW_GAINS);
  }

  @RequiredArgsConstructor
  public enum ArmGoal {
    START(
        new LoggedTunableNumber("Arm/Shoulder/Goal/StartHeightMeter", 0.4),
        new LoggedTunableNumber("Arm/Elbow/Goal/StartPositionDegree", -230.0)),
    IDLE(
        new LoggedTunableNumber("Arm/Shoulder/Goal/IdleHeightMeter", 0),
        new LoggedTunableNumber("Arm/Elbow/Goal/IdlePositionDegree", -90.0)),
    DEFENCE(
        new LoggedTunableNumber("Arm/Shoulder/Goal/DefenceHeightMeter", 0.3),
        new LoggedTunableNumber("Arm/Elbow/Goal/DefencePositionDegree", -250.0)),
    ALGAE_IDLE(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeIdleHeightMeter", 0.3),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeIdlePositionDegree", -90.0)),
    CORAL_STATION_PICK(
        new LoggedTunableNumber("Arm/Shoulder/Goal/CoralStationPickHeightMeter", 0.15),
        new LoggedTunableNumber("Arm/Elbow/Goal/CoralStationPickPositionDegree", -60.0)),
    CORAL_L1_SCORE(
        () -> shoulderCoralL1HeightMeter.get() + shoulderManualOffsetMeter,
        new LoggedTunableNumber("Arm/Elbow/Goal/CoralL1ScorePositionDegree", -190.0)),
    CORAL_L2_SCORE(
        () -> shoulderCoralL2HeightMeter.get() + shoulderManualOffsetMeter,
        new LoggedTunableNumber("Arm/Elbow/Goal/CoralL2ScorePositionDegree", -200.0)),
    CORAL_L3_SCORE(
        () -> shoulderCoralL3HeightMeter.get() + shoulderManualOffsetMeter,
        new LoggedTunableNumber("Arm/Elbow/Goal/CoralL3ScorePositionDegree", -200.0)),
    CORAL_L4_SCORE(
        () -> shoulderCoralL4HeightMeter.get() + shoulderManualOffsetMeter,
        new LoggedTunableNumber("Arm/Elbow/Goal/CoralL4ScorePositionDegree", -210.0)),
    CORAL_L4_AVOID_COLLISION(
        () -> shoulderCoralL4HeightMeter.get() + shoulderManualOffsetMeter,
        new LoggedTunableNumber("Arm/Elbow/Goal/CoralL4AvoidCollisionPositionDegree", -90)),
    ALGAE_GROUND_PICK(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeGroundPickHeightMeter", 0.2),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeGroundPickPositionDegree", 40.0)),
    ALGAE_LOW_PICK(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeLowPickHeightMeter", 0.2),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeLowPickPositionDegree", -131.0)),
    ALGAE_HIGH_PICK(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeHighPickHeightMeter", 0.6),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeHighPickPositionDegree", -132.0)),
    ALGAE_AUTO_LOW_PICK(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeAutoLowPickHeightMeter", 0.2),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeAutoLowPickPositionDegree", -131.0)),
    ALGAE_AUTO_HIGH_PICK(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeAutoHighPickHeightMeter", 0.6),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeAutoHighPickPositionDegree", -132.0)),
    ALGAE_PROCESSOR_SCORE(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeProcessorScoreHeightMeter", 0.0),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeProcessorScorePositionDegree", -180.0)),
    ALGAE_NET_SCORE(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeNetScoreHeightMeter", 1.5),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeNetScorePositionDegree", -45.0)),
    ALGAE_NET_PRESCORE(
        new LoggedTunableNumber("Arm/Shoulder/Goal/AlgaeNetPrescoreHeightMeter", 1.5),
        new LoggedTunableNumber("Arm/Elbow/Goal/AlgaeNetPrescorePositionDegree", -45.0)),
    CLIMB(
        new LoggedTunableNumber("Arm/Shoulder/Goal/ClimbHeightMeter", 0.4),
        new LoggedTunableNumber("Arm/Elbow/Goal/ClimbPositionDegree", -150.0)),
    HOME(
        new LoggedTunableNumber("Arm/Shoulder/Goal/HomeHeightMeter", 0.0),
        new LoggedTunableNumber("Arm/Elbow/Goal/HomePositionDegree", -90.0));

    private final DoubleSupplier shoulderHeightMeterSupplier;
    private final DoubleSupplier elbowPositionDegreeSupplier;

    double getShoulderHeightMeter() {
      return shoulderHeightMeterSupplier.getAsDouble();
    }

    double getElbowPositionRad() {
      return Units.degreesToRadians(elbowPositionDegreeSupplier.getAsDouble());
    }
  }

  private final DCMotorIO shoulderIO;
  private final DCMotorIO elbowIO;

  private final DCMotorIOInputsAutoLogged shoulderInputs = new DCMotorIOInputsAutoLogged();
  private final DCMotorIOInputsAutoLogged elbowInputs = new DCMotorIOInputsAutoLogged();

  private final AlertUtil shoulderOfflineAlert =
      new AlertUtil("Arm shoulder motor offline!", AlertUtil.AlertType.WARNING);
  private final AlertUtil elbowOfflineAlert =
      new AlertUtil("Arm elbow motor offline!", AlertUtil.AlertType.WARNING);

  @Getter @AutoLogOutput private ArmGoal goal = ArmGoal.START;
  private boolean needTransition = true;
  private boolean needGroundIntakeDodge = false;
  private boolean needAvoidReefAlgae = false;
  private boolean needShoulderFall = false;
  private boolean needElbowRotationPassDangerZone = false;

  private boolean isCharacterizing = false;
  private boolean isHoming = false;

  private boolean hasElbowTransitionPositionInit = false;

  private Debouncer homingDebouncer = new Debouncer(0.0);

  private static final double MAX_SHOULDER_MANUAL_OFFSET_METER = 0.1;
  private static double shoulderManualOffsetMeter = 0.0;

  private Arm(DCMotorIO shoulderIO, DCMotorIO elbowIO) {
    this.shoulderIO = shoulderIO;
    this.elbowIO = elbowIO;
  }

  @Override
  public void periodic() {
    updateInputs();

    // if (this.goal == ArmGoal.ALGAE_NET_PRESCORE && this.atGoal()) {
    // setGoal(ArmGoal.ALGAE_NET_SCORE);
    // }

    if (!hasElbowTransitionPositionInit) {
      hasElbowTransitionPositionInit = true;
      elbowIO.setAppliedPositionF(elbowInputs.appliedPosition, 0.0);
    }

    shoulderOfflineAlert.set(!shoulderInputs.connected);
    elbowOfflineAlert.set(!elbowInputs.connected);

    LoggedTunableGains.ifChanged(
        hashCode(), () -> shoulderIO.setPidsg(shoulderGains.get()), shoulderGains);
    LoggedTunableGains.ifChanged(hashCode(), () -> elbowIO.setPidsg(elbowGains.get()), elbowGains);

    if (isCharacterizing || isHoming) {
      return;
    }

    Logger.recordOutput("Arm/ElbowGoal", goal.getElbowPositionRad());
    Logger.recordOutput("Arm/ShoulderGoal", goal.getShoulderHeightMeter());
    Logger.recordOutput("Arm/ElbowAtGoal", elbowAtGoal());
    Logger.recordOutput("Arm/ShoulderAtGoal", shoulderAtGoal());

    var minSafeGroundIntakeDodgeElevatorHeightMeterVal =
        minSafeGroundIntakeDodgeElevatorHeightMeter.get();

    if (shoulderInputs.appliedPosition >= minSafeGroundIntakeDodgeElevatorHeightMeterVal
        && goal.getShoulderHeightMeter() >= minSafeGroundIntakeDodgeElevatorHeightMeterVal) {
      needGroundIntakeDodge = false;
    } else {
      needGroundIntakeDodge =
          // isRotationEnterSpecificAngleArea(
          // elbowInputs.positionRad,
          // goal.getElbowPositionRad(),
          // Units.degreesToRadians(150.0),
          // Units.degreesToRadians(255.0))
          // || isRotationEnterSpecificAngleArea(
          // elbowInputs.positionRad,
          // goal.getElbowPositionRad(),
          // Units.degreesToRadians(-210.0),
          // Units.degreesToRadians(-105.0))
          // ||
          isRotationEnterSpecificAngleArea(
              elbowInputs.appliedPosition,
              goal.getElbowPositionRad(),
              Units.degreesToRadians(-22.5),
              Units.degreesToRadians(90.0));
    }

    var elbowAvoidReefAlgaePositionRad =
        Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get());

    if (needTransition) {
      shoulderIO.setAppliedPositionF(transitionElevatorHeightMeter.get(), 0.0);

      if ((needAvoidReefAlgae) && !needElbowRotationPassDangerZone) {
        elbowIO.setAppliedPositionF(elbowAvoidReefAlgaePositionRad, 0.0);
      }

      if (shoulderAtTransition()) {
        if (needShoulderFall) {
          needAvoidReefAlgae = false;
          elbowIO.setAppliedPositionF(goal.getElbowPositionRad(), 0.0);

          if (elbowAtGoal()) {
            needTransition = false;
          }
        } else if (needAvoidReefAlgae) {
          elbowIO.setAppliedPositionF(elbowAvoidReefAlgaePositionRad, 0.0);

          if (elbowAtPosition(elbowAvoidReefAlgaePositionRad)) {
            needTransition = false;
          }
        } else {
          elbowIO.setAppliedPositionF(goal.getElbowPositionRad(), 0.0);

          if (elbowAtGoal()) {
            needTransition = false;
          }
        }
      }
    } else {
      shoulderIO.setAppliedPositionF(goal.getShoulderHeightMeter(), 0.0);

      if (needAvoidReefAlgae) {
        elbowIO.setAppliedPositionF(elbowAvoidReefAlgaePositionRad, 0.0);

        if (elbowAtPosition(elbowAvoidReefAlgaePositionRad) && shoulderAtGoal()) {
          needAvoidReefAlgae = false;
        }
      } else {
        elbowIO.setAppliedPositionF(goal.getElbowPositionRad(), 0.0);
      }
    }
  }

  private void updateInputs() {
    shoulderIO.updateInputs(shoulderInputs);
    Logger.processInputs("Arm Shoulder", shoulderInputs);

    elbowIO.updateInputs(elbowInputs);
    Logger.processInputs("Arm Elbow", elbowInputs);
  }

  public void setGoal(ArmGoal goal) {
    if (this.goal != goal) {
      if (goal == ArmGoal.CORAL_L2_SCORE
          || goal == ArmGoal.CORAL_L3_SCORE
          || goal == ArmGoal.CORAL_L4_SCORE
          || this.goal == ArmGoal.CORAL_L2_SCORE
          || this.goal == ArmGoal.CORAL_L3_SCORE
          || this.goal == ArmGoal.CORAL_L4_SCORE) {
        needAvoidReefAlgae = false;
      }

      needShoulderFall =
          shoulderInputs.appliedPosition > transitionElevatorHeightMeter.getAsDouble()
              && goal.getShoulderHeightMeter() <= transitionElevatorHeightMeter.getAsDouble();

      needElbowRotationPassDangerZone =
          isRotationEnterSpecificAngleArea(
              elbowInputs.appliedPosition,
              needAvoidReefAlgae
                  ? Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get())
                  : goal.getElbowPositionRad(),
              Units.degreesToRadians(-120.0),
              Units.degreesToRadians(-90.0));

      if ((this.goal == ArmGoal.ALGAE_LOW_PICK || this.goal == ArmGoal.ALGAE_HIGH_PICK)) {
        needTransition = !needShoulderFall;
      } else {
        var needMoveElbowAtBottomAndPassSpecificAngle =
            shoulderInputs.appliedPosition < transitionElevatorHeightMeter.getAsDouble();

        needTransition = (needMoveElbowAtBottomAndPassSpecificAngle || needShoulderFall);
      }

      if (needTransition) {
        elbowIO.setAppliedPositionF(elbowInputs.appliedPosition, 0.0);
      }

      // if (goal == ArmGoal.ALGAE_NET_SCORE) {
      // this.goal = ArmGoal.ALGAE_NET_PRESCORE;
      // return;
      // }

      this.goal = goal;
    }
  }

  public boolean atGoal() {
    return atGoalInternal(false);
  }

  @AutoLogOutput(key = "Arm/StopAtGoal")
  public boolean stopAtGoal() {
    return atGoalInternal(true);
  }

  private boolean atGoalInternal(boolean needStop) {
    var atEndPosition = shoulderAtGoal() && elbowAtGoal();

    if (needStop) {
      var hasStop =
          EqualsUtil.epsilonEquals(
                  0.0, shoulderInputs.appliedVelocity, shoulderStopToleranceMeterPerSec.get())
              && EqualsUtil.epsilonEquals(
                  0.0,
                  elbowInputs.appliedVelocity,
                  Units.degreesToRadians(elbowStopToleranceDegreePerSec.get()));

      return !needTransition && !needAvoidReefAlgae && atEndPosition && hasStop;
    } else {
      return !needTransition && !needAvoidReefAlgae && atEndPosition;
    }
  }

  public boolean shoulderAtGoal() {
    return shoulderAtPosition(goal.getShoulderHeightMeter());
  }

  public boolean elbowAtGoal() {
    return elbowAtPosition(goal.getElbowPositionRad());
  }

  private boolean shoulderAtTransition() {
    return shoulderAtPosition(transitionElevatorHeightMeter.get());
  }

  private boolean shoulderAtPosition(double positionMeter) {
    return EqualsUtil.epsilonEquals(
        positionMeter, shoulderInputs.appliedPosition, shoulderToleranceMeter.get());
  }

  private boolean elbowAtPosition(double positionRad) {
    return EqualsUtil.epsilonEquals(
        positionRad,
        elbowInputs.appliedPosition,
        Units.degreesToRadians(elbowToleranceDegree.get()));
  }

  public boolean needGroundIntakeDodge() {
    return needGroundIntakeDodge;
  }

  public void stop() {
    shoulderIO.stop();
    elbowIO.stop();
  }

  private boolean isRotationEnterSpecificAngleArea(
      double startAngleRad,
      double endAngleRad,
      double minSpecificAngleRad,
      double maxSpecificAngleRad) {
    return (startAngleRad <= maxSpecificAngleRad && startAngleRad >= minSpecificAngleRad)
        || (endAngleRad <= maxSpecificAngleRad && endAngleRad >= minSpecificAngleRad)
        || isRotationPassSpecificAngle(startAngleRad, endAngleRad, minSpecificAngleRad)
        || isRotationPassSpecificAngle(startAngleRad, endAngleRad, maxSpecificAngleRad);
  }

  private boolean isRotationPassSpecificAngle(
      double startAngleRad, double endAngleRad, double specificAngleRad) {
    return ((startAngleRad < specificAngleRad) && (endAngleRad > specificAngleRad))
        || ((startAngleRad > specificAngleRad) && (endAngleRad < specificAngleRad));
  }

  // public void setGoalBySelection(
  // GamePiece.GamePieceType selectedGamePieceType, String selectedBranch, String
  // selectedLevel) {
  // setGoal(
  // switch (selectedGamePieceType) {
  // case ALGAE -> switch (selectedBranch) {
  // case "N" -> Arm.ArmGoal.ALGAE_NET_SCORE;
  // case "P" -> Arm.ArmGoal.ALGAE_PROCESSOR_SCORE;
  // case "A", "B", "E", "F", "I", "J", "AB", "EF", "IJ" ->
  // Arm.ArmGoal.ALGAE_HIGH_PICK;
  // case "C", "D", "G", "H", "K", "L", "CD", "GH", "KL" ->
  // Arm.ArmGoal.ALGAE_LOW_PICK;
  // default -> goal;
  // };

  // case CORAL -> switch (selectedLevel) {
  // case "1" -> Arm.ArmGoal.CORAL_L1_SCORE;
  // case "2" -> Arm.ArmGoal.CORAL_L2_SCORE;
  // case "3" -> Arm.ArmGoal.CORAL_L3_SCORE;
  // case "4" -> Arm.ArmGoal.CORAL_L4_SCORE;
  // default -> goal;
  // };
  // });
  // }

  public double getCOGHeightPercent() {
    return MathUtil.clamp(
        shoulderInputs.appliedPosition / ArmConfig.SHOULDER_MAX_HEIGHT_METER, 0.0, 1.0);
  }

  public Command getShoulderKsCharacterizationCmd(double outputCurrentRampRateAmp) {
    final var state = new StaticCharacterizationState();
    var timer = new Timer();

    return Commands.startRun(
            () -> {
              isCharacterizing = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputCurrentRampRateAmp * timer.get();
              shoulderIO.setCurrent(state.characterizationOutput);
              Logger.recordOutput(
                  "Arm/Shoulder/StaticCharacterizationCurrentOutputAmp",
                  state.characterizationOutput);
            },
            this)
        .until(
            () ->
                shoulderInputs.appliedVelocity
                    >= shoulderStaticCharacterizationVelocityThreshMeterPerSec.get())
        .finallyDo(
            () -> {
              isCharacterizing = false;
              shoulderIO.setCurrent(0.0);
              timer.stop();
              Logger.recordOutput(
                  "Arm/Shoulder/StaticCharacterizationCurrentOutputAmp",
                  state.characterizationOutput);

              System.out.println("Calculated Ks: " + state.characterizationOutput + " amps");
            })
        .withName("[Arm] Shoulder kS Characterization");
  }

  public Command getElbowKsCharacterizationCmd(double outputCurrentRampRateAmp) {
    final var state = new StaticCharacterizationState();
    var timer = new Timer();

    return Commands.startRun(
            () -> {
              isCharacterizing = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputCurrentRampRateAmp * timer.get();
              elbowIO.setCurrent(state.characterizationOutput);
              Logger.recordOutput(
                  "Arm/Elbow/StaticCharacterizationCurrentOutputAmp", state.characterizationOutput);
            },
            this)
        .until(
            () ->
                elbowInputs.appliedVelocity
                    >= Units.degreesToRadians(
                        elbowStaticCharacterizationVelocityThreshDegreePerSec.get()))
        .finallyDo(
            () -> {
              isCharacterizing = false;
              elbowIO.setCurrent(0.0);
              timer.stop();
              Logger.recordOutput(
                  "Arm/Elbow/StaticCharacterizationCurrentOutputAmp", state.characterizationOutput);

              System.out.println("Calculated Ks: " + state.characterizationOutput + " amps");
            })
        .withName("[Arm] Elbow kS Characterization");
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public Command getHomeCmd() {
    BooleanSupplier hasStall =
        () ->
            homingDebouncer.calculate(
                Math.abs(shoulderInputs.appliedVelocity)
                    <= shoulderHomingVelocityThreshMeterPerSec.get());

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  shoulderManualOffsetMeter = 0.0;

                  setGoal(ArmGoal.HOME);
                  homingDebouncer = new Debouncer(shoulderHomingTimeSecs.get());
                  homingDebouncer.calculate(false);
                }),
            Commands.waitUntil(() -> this.atGoal() || (!this.atGoal() && hasStall.getAsBoolean())),
            Commands.startRun(
                    () -> isHoming = true,
                    () -> shoulderIO.setCurrent(shoulderHomingCurrentAmp.get()),
                    this)
                .until(hasStall),
            Commands.runOnce(() -> shoulderIO.resetAppliedPosition(0.0)))
        .finallyDo(
            () -> {
              setGoal(ArmGoal.IDLE);
              isHoming = false;
            })
        .withName("[Arm] Shoulder Offset Calibrate");
  }

  public void increaseShoulderManualOffsetMeter(double val) {
    shoulderManualOffsetMeter =
        MathUtil.clamp(
            shoulderManualOffsetMeter + val,
            -MAX_SHOULDER_MANUAL_OFFSET_METER,
            MAX_SHOULDER_MANUAL_OFFSET_METER);
  }

  public void decreaseShoulderManualOffsetMeter(double val) {
    shoulderManualOffsetMeter =
        MathUtil.clamp(
            shoulderManualOffsetMeter - val,
            -MAX_SHOULDER_MANUAL_OFFSET_METER,
            MAX_SHOULDER_MANUAL_OFFSET_METER);
  }

  public Rotation2d getElbowRotation() {
    return new Rotation2d(elbowInputs.appliedPosition);
  }

  public Boolean isShoulderOverHeight() {
    return shoulderInputs.appliedPosition >= 1.3;
  }

  public static Arm createSim() {
    return new Arm(
        new DCMotorIOSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                7.5,
                ArmConfig.SHOULDER_METER_PER_ROTATION / Math.PI / 2.0,
                ArmConfig.SHOULDER_REDUCTION),
            DCMotor.getKrakenX60(2),
            UnitConverter.scale(ArmConfig.SHOULDER_METER_PER_ROTATION / Math.PI / 2.0)
                .withUnits("rad", "m")),
        new DCMotorIOSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), 0.46, ArmConfig.ELBOW_REDUCTION),
            DCMotor.getKrakenX60(1),
            UnitConverter.identity().withUnits("rad", "rad")));
  }

  public static Arm createReal() {
    return new Arm(
        new DCMotorIOTalonfx(
                "ArmShoulder",
                Constants.Ports.Can.ARM_SHOULDER_MASTER,
                ArmConfig.getShoulderTalonConfig(),
                UnitConverter.identity().withUnits("rot", "m"))
            .withFollower(Constants.Ports.Can.ARM_SHOULDER_SLAVE, true),
        new DCMotorIOTalonfxCancoder(
            "ArmElbow",
            Constants.Ports.Can.ARM_ELBOW,
            ArmConfig.getElbowTalonConfig(),
            Constants.Ports.Can.ARM_ELBOW_CANCODER,
            ArmConfig.getCancoderConfig(-0.2958333 + 0.5),
            UnitConverter.scale(Math.PI * 2.0).withUnits("rot", "rad"),
            UnitConverter.offset(-0.2958333 + 0.5).withUnits("rot", "rot")));
  }

  public static Arm createIO() {
    return new Arm(new DCMotorIO() {}, new DCMotorIO() {});
  }
}
