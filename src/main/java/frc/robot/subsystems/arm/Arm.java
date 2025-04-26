package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.services.VisualizeService;
import frc.robot.utils.dashboard.AlertManager;
import frc.robot.utils.dashboard.TunableNumbers;
import frc.robot.utils.math.EqualsUtil;
import frc.robot.utils.math.UnitConverter;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final DCMotorIO shoulderIO;
  private final DCMotorIO elbowIO;

  private final DCMotorIOInputsAutoLogged shoulderInputs = new DCMotorIOInputsAutoLogged();
  private final DCMotorIOInputsAutoLogged elbowInputs = new DCMotorIOInputsAutoLogged();

  private final AlertManager shoulderOfflineAlert =
      new AlertManager("Arm shoulder motor offline!", AlertManager.AlertType.WARNING);
  private final AlertManager elbowOfflineAlert =
      new AlertManager("Arm elbow motor offline!", AlertManager.AlertType.WARNING);

  @Getter private static ArmGoal goal = ArmGoal.START;

  private boolean needTransition = true;
  private boolean needGroundIntakeDodge = false;
  private boolean needAvoidReefAlgae = false;
  private boolean needShoulderFall = false;
  private boolean needElbowRotationPassDangerZone = false;

  private boolean isCharacterizing = false;
  private boolean isHoming = false;

  private boolean hasElbowTransitionPositionInit = false;

  private Debouncer homingDebouncer = new Debouncer(0.0);

  private Arm(DCMotorIO shoulderIO, DCMotorIO elbowIO) {
    this.shoulderIO = shoulderIO;
    this.elbowIO = elbowIO;
  }

  @Override
  public void periodic() {
    updateInputs();

    if (!hasElbowTransitionPositionInit) {
      hasElbowTransitionPositionInit = true;
      elbowIO.setAppliedPositionF(elbowInputs.appliedPosition, 0.0);
    }

    shoulderOfflineAlert.set(!shoulderInputs.connected);
    elbowOfflineAlert.set(!elbowInputs.connected);

    TunableNumbers.ifChanged(
        this.hashCode(),
        () -> shoulderIO.setGains(ArmConfig.SHOULDER_GAINS),
        ArmConfig.SHOULDER_GAINS);
    TunableNumbers.ifChanged(
        this.hashCode(), () -> elbowIO.setGains(ArmConfig.ELBOW_GAINS), ArmConfig.ELBOW_GAINS);

    if (isCharacterizing || isHoming) {
      return;
    }

    Logger.recordOutput("Arm/ElbowGoal", goal.getElbowPositionDegree());
    Logger.recordOutput("Arm/ShoulderGoal", goal.getShoulderHeightMeter());
    Logger.recordOutput("Arm/ElbowAtGoal", elbowAtGoal());
    Logger.recordOutput("Arm/ShoulderAtGoal", shoulderAtGoal());

    var minSafeGroundIntakeDodgeElevatorHeightMeterVal =
        ArmConfig.minSafeGroundIntakeDodgeElevatorHeightMeter.get();

    if (shoulderInputs.appliedPosition >= minSafeGroundIntakeDodgeElevatorHeightMeterVal
        && goal.getShoulderHeightMeter() >= minSafeGroundIntakeDodgeElevatorHeightMeterVal) {
      needGroundIntakeDodge = false;
    } else {
      needGroundIntakeDodge =
          isRotationEnterSpecificAngleArea(
                  elbowInputs.appliedPosition, goal.getElbowPositionDegree(), 150, 255)
              || isRotationEnterSpecificAngleArea(
                  elbowInputs.appliedPosition, goal.getElbowPositionDegree(), -210, -105)
              || isRotationEnterSpecificAngleArea(
                  elbowInputs.appliedPosition, goal.getElbowPositionDegree(), -22.5, 90);
    }

    var elbowAvoidReefAlgaePositionDegree = ArmConfig.elbowAvoidReefAlgaePositionDegree.get();

    if (needTransition) {
      shoulderIO.setAppliedPositionF(ArmConfig.transitionElevatorHeightMeter.get(), 0.0);

      if ((needAvoidReefAlgae) && !needElbowRotationPassDangerZone) {
        elbowIO.setAppliedPositionF(elbowAvoidReefAlgaePositionDegree, 0.0);
      }

      if (shoulderAtTransition()) {
        if (needShoulderFall) {
          needAvoidReefAlgae = false;
          elbowIO.setAppliedPositionF(goal.getElbowPositionDegree(), 0.0);

          if (elbowAtGoal()) {
            needTransition = false;
          }
        } else if (needAvoidReefAlgae) {
          elbowIO.setAppliedPositionF(elbowAvoidReefAlgaePositionDegree, 0.0);

          if (elbowAtPosition(elbowAvoidReefAlgaePositionDegree)) {
            needTransition = false;
          }
        } else {
          elbowIO.setAppliedPositionF(goal.getElbowPositionDegree(), 0.0);

          if (elbowAtGoal()) {
            needTransition = false;
          }
        }
      }
    } else {
      shoulderIO.setAppliedPositionF(goal.getShoulderHeightMeter(), 0.0);

      if (needAvoidReefAlgae) {
        elbowIO.setAppliedPositionF(elbowAvoidReefAlgaePositionDegree, 0.0);

        if (elbowAtPosition(elbowAvoidReefAlgaePositionDegree) && shoulderAtGoal()) {
          needAvoidReefAlgae = false;
        }
      } else {
        elbowIO.setAppliedPositionF(goal.getElbowPositionDegree(), 0.0);
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
          shoulderInputs.appliedPosition > ArmConfig.transitionElevatorHeightMeter.getAsDouble()
              && goal.getShoulderHeightMeter()
                  <= ArmConfig.transitionElevatorHeightMeter.getAsDouble();

      needElbowRotationPassDangerZone =
          isRotationEnterSpecificAngleArea(
              elbowInputs.appliedPosition,
              needAvoidReefAlgae
                  ? ArmConfig.elbowAvoidReefAlgaePositionDegree.get()
                  : goal.getElbowPositionDegree(),
              -120,
              -90);

      if ((this.goal == ArmGoal.ALGAE_LOW_PICK || this.goal == ArmGoal.ALGAE_HIGH_PICK)) {
        needTransition = !needShoulderFall;
      } else {
        var needMoveElbowAtBottomAndPassSpecificAngle =
            shoulderInputs.appliedPosition < ArmConfig.transitionElevatorHeightMeter.getAsDouble();

        needTransition = (needMoveElbowAtBottomAndPassSpecificAngle || needShoulderFall);
      }

      if (needTransition) {
        elbowIO.setAppliedPositionF(elbowInputs.appliedPosition, 0.0);
      }

      this.goal = goal;
      Logger.recordOutput("Arm/Goal", goal.getName());
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
                  0.0,
                  shoulderInputs.appliedVelocity,
                  ArmConfig.shoulderStopToleranceMeterPerSec.get())
              && EqualsUtil.epsilonEquals(
                  0.0, elbowInputs.appliedVelocity, ArmConfig.elbowStopToleranceDegreePerSec.get());

      return !needTransition && !needAvoidReefAlgae && atEndPosition && hasStop;
    } else {
      return !needTransition && !needAvoidReefAlgae && atEndPosition;
    }
  }

  public boolean shoulderAtGoal() {
    return shoulderAtPosition(goal.getShoulderHeightMeter());
  }

  public boolean elbowAtGoal() {
    return elbowAtPosition(goal.getElbowPositionDegree());
  }

  private boolean shoulderAtTransition() {
    return shoulderAtPosition(ArmConfig.transitionElevatorHeightMeter.get());
  }

  private boolean shoulderAtPosition(double positionMeter) {
    return EqualsUtil.epsilonEquals(
        positionMeter, shoulderInputs.appliedPosition, ArmConfig.shoulderToleranceMeter.get());
  }

  private boolean elbowAtPosition(double positionDegree) {
    return EqualsUtil.epsilonEquals(
        positionDegree, elbowInputs.appliedPosition, ArmConfig.elbowToleranceDegree.get());
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
                    >= ArmConfig.shoulderStaticCharacterizationVelocityThreshMeterPerSec.get())
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
        .withName("Arm/Shoulder kS Characterization");
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
                    >= ArmConfig.elbowStaticCharacterizationVelocityThreshDegreePerSec.get())
        .finallyDo(
            () -> {
              isCharacterizing = false;
              elbowIO.setCurrent(0.0);
              timer.stop();
              Logger.recordOutput(
                  "Arm/Elbow/StaticCharacterizationCurrentOutputAmp", state.characterizationOutput);

              System.out.println("Calculated Ks: " + state.characterizationOutput + " amps");
            })
        .withName("Arm/Elbow kS Characterization");
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public Command getHomeCmd() {
    BooleanSupplier hasStall =
        () ->
            homingDebouncer.calculate(
                Math.abs(shoulderInputs.appliedVelocity)
                    <= ArmConfig.shoulderHomingVelocityThreshMeterPerSec.get());

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  setGoal(ArmGoal.HOME);
                  homingDebouncer = new Debouncer(ArmConfig.shoulderHomingTimeSecs.get());
                  homingDebouncer.calculate(false);
                }),
            Commands.waitUntil(() -> this.atGoal() || (!this.atGoal() && hasStall.getAsBoolean())),
            Commands.startRun(
                    () -> isHoming = true,
                    () -> shoulderIO.setCurrent(ArmConfig.shoulderHomingCurrentAmp.get()),
                    this)
                .until(hasStall),
            Commands.runOnce(() -> shoulderIO.resetAppliedPosition(0.0)))
        .finallyDo(
            () -> {
              setGoal(ArmGoal.IDLE);
              isHoming = false;
            })
        .withName("Arm/Shoulder Offset Calibrate");
  }

  public static Arm createSim() {
    return new Arm(
        new DCMotorIOSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                5.0,
                ArmConfig.SHOULDER_METER_PER_ROTATION / Math.PI / 2.0,
                ArmConfig.SHOULDER_REDUCTION / ArmConfig.SHOULDER_REDUCTION),
            DCMotor.getKrakenX60(2),
            UnitConverter.scale(1.0 / Math.PI / 2.0).withUnits("rad", "m"),
            UnitConverter.offset(goal.getShoulderHeightMeter()).withUnits("m", "m"),
            ArmConfig.SHOULDER_GAINS,
            0,
            ArmConfig.SHOULDER_MAX_HEIGHT_METER),
        new DCMotorIOSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), 0.46, ArmConfig.ELBOW_REDUCTION),
            DCMotor.getKrakenX60(1),
            UnitConverter.scale(180.0 / Math.PI).withUnits("rad", "deg"),
            UnitConverter.offset(goal.getElbowPositionDegree()).withUnits("deg", "deg"),
            ArmConfig.ELBOW_GAINS,
            -360,
            360));
  }

  public static Arm createReal() {
    return new Arm(
        new DCMotorIOTalonfx(
                "ArmShoulder",
                Constants.Ports.Can.ARM_SHOULDER_MASTER,
                ArmConfig.getShoulderTalonConfig(),
                UnitConverter.scale(ArmConfig.SHOULDER_METER_PER_ROTATION).withUnits("rot", "m"),
                UnitConverter.offset(goal.getShoulderHeightMeter()).withUnits("m", "m"))
            .withFollower(Constants.Ports.Can.ARM_SHOULDER_SLAVE, true),
        new DCMotorIOTalonfxCancoder(
            "ArmElbow",
            Constants.Ports.Can.ARM_ELBOW,
            ArmConfig.getElbowTalonConfig(),
            Constants.Ports.Can.ARM_ELBOW_CANCODER,
            ArmConfig.getCancoderConfig(-0.2958333 + 0.5),
            UnitConverter.scale(360).withUnits("rot", "deg"),
            UnitConverter.offset(360 * (-0.2958333 + 0.5)).withUnits("deg", "deg")));
  }

  public static Arm createIO() {
    return new Arm(new DCMotorIO() {}, new DCMotorIO() {});
  }

  public void registerVisualize(VisualizeService visualizer) {
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.ELEVATOR_L2,
        Constants.Ascope.Component.DRIVETRAIN,
        () -> {
          return ArmConfig.L2_ZEROED_TF.plus(
              new Transform3d(
                  0,
                  MathUtil.clamp(
                      shoulderInputs.appliedPosition
                          - ArmConfig.SHOULDER_SINGLE_LEVEL_HEIGHT_METER * 2.0,
                      0,
                      ArmConfig.SHOULDER_SINGLE_LEVEL_HEIGHT_METER),
                  0,
                  new Rotation3d()));
        });

    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.ELEVATOR_L3,
        Constants.Ascope.Component.ELEVATOR_L2,
        () -> {
          return ArmConfig.L3_ZEROED_TF.plus(
              new Transform3d(
                  0,
                  MathUtil.clamp(
                      shoulderInputs.appliedPosition - ArmConfig.SHOULDER_SINGLE_LEVEL_HEIGHT_METER,
                      0,
                      ArmConfig.SHOULDER_SINGLE_LEVEL_HEIGHT_METER),
                  0,
                  new Rotation3d()));
        });

    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.ELEVATOR_CARRIAGE,
        Constants.Ascope.Component.ELEVATOR_L3,
        () -> {
          return ArmConfig.L4_ZEROED_TF.plus(
              new Transform3d(
                  0,
                  MathUtil.clamp(
                      shoulderInputs.appliedPosition,
                      0,
                      ArmConfig.SHOULDER_SINGLE_LEVEL_HEIGHT_METER),
                  0,
                  new Rotation3d()));
        });

    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.ARM,
        Constants.Ascope.Component.ELEVATOR_CARRIAGE,
        () -> {
          return ArmConfig.ARM_ZEROED_TF.plus(
              new Transform3d(
                  new Translation3d(),
                  new Rotation3d(Units.degreesToRadians(elbowInputs.appliedPosition), 0, 0)));
        });
  }
}
