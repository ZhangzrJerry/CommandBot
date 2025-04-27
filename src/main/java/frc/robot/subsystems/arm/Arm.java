package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CharacterizationCommandFactory;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonFX;
import frc.robot.services.TransformTree;
import frc.robot.utils.dashboard.TunableNumbers;
import frc.robot.utils.math.EqualsUtil;
import frc.robot.utils.math.UnitConverter;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Arm subsystem, controls arm movement */
public class Arm extends SubsystemBase {
  private final DCMotorIO shoulderIO;
  private final DCMotorIO elbowIO;

  private final DCMotorIOInputsAutoLogged shoulderInputs = new DCMotorIOInputsAutoLogged();
  private final DCMotorIOInputsAutoLogged elbowInputs = new DCMotorIOInputsAutoLogged();

  @Getter private ArmGoal goal = ArmGoal.START;

  // State flags
  private boolean needTransition = true;
  private boolean needGroundIntakeDodge = false;
  private boolean needAvoidReefAlgae = false;
  private boolean needShoulderFall = false;
  private boolean needElbowRotationPassDangerZone = false;
  private boolean isCharacterizing = false;
  private boolean isHoming = false;
  private boolean hasElbowTransitionPositionInit = false;

  private Debouncer homingDebouncer = new Debouncer(0.0);

  /** Constructor */
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

    // Update PID gains
    TunableNumbers.ifChanged(
        this.hashCode(),
        () -> shoulderIO.setGains(ArmConfig.SHOULDER_GAINS),
        ArmConfig.SHOULDER_GAINS);
    TunableNumbers.ifChanged(
        this.hashCode(), () -> elbowIO.setGains(ArmConfig.ELBOW_GAINS), ArmConfig.ELBOW_GAINS);

    if (isCharacterizing || isHoming) {
      return;
    }

    // Log data
    Logger.recordOutput("Arm/ElbowGoal", goal.getElbowPositionDegree());
    Logger.recordOutput("Arm/ShoulderGoal", goal.getShoulderHeightMeter());
    Logger.recordOutput("Arm/ElbowAtGoal", elbowAtGoal());
    Logger.recordOutput("Arm/ShoulderAtGoal", shoulderAtGoal());

    // Update state and control movement
    updateAndControl();
  }

  /** Update input data */
  private void updateInputs() {
    shoulderIO.updateInputs(shoulderInputs);
    Logger.processInputs("Arm/Shoulder", shoulderInputs);

    elbowIO.updateInputs(elbowInputs);
    Logger.processInputs("Arm/Elbow", elbowInputs);
  }

  /** Update state and control movement */
  private void updateAndControl() {
    // Update state
    var minSafeGroundIntakeDodgeElevatorHeightMeterVal =
        ArmConfig.MIN_SAFE_GROUND_INTAKE_DODGE_ELEVATOR_HEIGHT_METER.get();
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

    // Control movement
    if (needTransition) {
      // Transition movement
      shoulderIO.setAppliedPositionF(ArmConfig.TRANSITION_ELEVATOR_HEIGHT_METER.get(), 0.0);

      if (needAvoidReefAlgae && !needElbowRotationPassDangerZone) {
        elbowIO.setAppliedPositionF(ArmConfig.ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE.get(), 0.0);
      }

      if (shoulderAtTransition()) {
        if (needShoulderFall) {
          needAvoidReefAlgae = false;
          elbowIO.setAppliedPositionF(goal.getElbowPositionDegree(), 0.0);
          if (elbowAtGoal()) {
            needTransition = false;
          }
        } else if (needAvoidReefAlgae) {
          elbowIO.setAppliedPositionF(ArmConfig.ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE.get(), 0.0);
          if (elbowAtPosition(ArmConfig.ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE.get())) {
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
      // Normal movement
      shoulderIO.setAppliedPositionF(goal.getShoulderHeightMeter(), 0.0);

      if (needAvoidReefAlgae) {
        elbowIO.setAppliedPositionF(ArmConfig.ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE.get(), 0.0);
        if (elbowAtPosition(ArmConfig.ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE.get())
            && shoulderAtGoal()) {
          needAvoidReefAlgae = false;
        }
      } else {
        elbowIO.setAppliedPositionF(goal.getElbowPositionDegree(), 0.0);
      }
    }
  }

  /** Set target position */
  public void setGoal(ArmGoal goal) {
    if (this.goal != goal) {
      // Update target state
      if (isCoralGoal(goal) || isCoralGoal(this.goal)) {
        needAvoidReefAlgae = false;
      }

      needShoulderFall =
          shoulderInputs.appliedPosition > ArmConfig.TRANSITION_ELEVATOR_HEIGHT_METER.getAsDouble()
              && goal.getShoulderHeightMeter()
                  <= ArmConfig.TRANSITION_ELEVATOR_HEIGHT_METER.getAsDouble();

      needElbowRotationPassDangerZone =
          isRotationEnterSpecificAngleArea(
              elbowInputs.appliedPosition,
              needAvoidReefAlgae
                  ? ArmConfig.ELBOW_AVOID_REEF_ALGAE_POSITION_DEGREE.get()
                  : goal.getElbowPositionDegree(),
              -120,
              -90);

      // Update transition state
      if (isAlgaePickGoal(this.goal)) {
        needTransition = !needShoulderFall;
      } else {
        var needMoveElbowAtBottomAndPassSpecificAngle =
            shoulderInputs.appliedPosition
                < ArmConfig.TRANSITION_ELEVATOR_HEIGHT_METER.getAsDouble();
        needTransition = (needMoveElbowAtBottomAndPassSpecificAngle || needShoulderFall);
      }

      if (needTransition) {
        elbowIO.setAppliedPositionF(elbowInputs.appliedPosition, 0.0);
      }

      this.goal = goal;
      Logger.recordOutput("Arm/Goal", goal.getName());
    }
  }

  /** Check if it's a coral goal */
  private boolean isCoralGoal(ArmGoal goal) {
    return goal == ArmGoal.CORAL_L2_SCORE
        || goal == ArmGoal.CORAL_L3_SCORE
        || goal == ArmGoal.CORAL_L4_SCORE;
  }

  /** Check if it's an algae pick goal */
  private boolean isAlgaePickGoal(ArmGoal goal) {
    return goal == ArmGoal.ALGAE_LOW_PICK || goal == ArmGoal.ALGAE_HIGH_PICK;
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
                  ArmConfig.SHOULDER_STOP_TOLERANCE_METER_PER_SEC.get())
              && EqualsUtil.epsilonEquals(
                  0.0,
                  elbowInputs.appliedVelocity,
                  ArmConfig.ELBOW_STOP_TOLERANCE_DEGREE_PER_SEC.get());

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
    return shoulderAtPosition(ArmConfig.TRANSITION_ELEVATOR_HEIGHT_METER.get());
  }

  private boolean shoulderAtPosition(double positionMeter) {
    return EqualsUtil.epsilonEquals(
        positionMeter, shoulderInputs.appliedPosition, ArmConfig.SHOULDER_TOLERANCE_METER.get());
  }

  private boolean elbowAtPosition(double positionDegree) {
    return EqualsUtil.epsilonEquals(
        positionDegree, elbowInputs.appliedPosition, ArmConfig.ELBOW_TOLERANCE_DEGREE.get());
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
    return CharacterizationCommandFactory.createKsCharacterizationCommand(
        "Arm/Shoulder",
        () -> outputCurrentRampRateAmp,
        () -> ArmConfig.SHOULDER_STATIC_CHARACTERIZATION_VELOCITY_THRESH_METER_PER_SEC.get(),
        current -> {
          isCharacterizing = true;
          shoulderIO.setCurrent(current);
        },
        () -> shoulderInputs.appliedVelocity);
  }

  public Command getElbowKsCharacterizationCmd(double outputCurrentRampRateAmp) {
    return CharacterizationCommandFactory.createKsCharacterizationCommand(
        "Arm/Elbow",
        () -> outputCurrentRampRateAmp,
        () -> ArmConfig.ELBOW_STATIC_CHARACTERIZATION_VELOCITY_THRESH_DEGREE_PER_SEC.get(),
        current -> {
          isCharacterizing = true;
          elbowIO.setCurrent(current);
        },
        () -> elbowInputs.appliedVelocity);
  }

  public Command getHomeCmd() {
    BooleanSupplier hasStall =
        () ->
            homingDebouncer.calculate(
                Math.abs(shoulderInputs.appliedVelocity)
                    <= ArmConfig.SHOULDER_HOMING_VELOCITY_THRESH_METER_PER_SEC.get());
    String name = "Arm/Home";
    return Commands.sequence(
            Commands.runOnce(
                    () -> {
                      setGoal(ArmGoal.HOME);
                      homingDebouncer = new Debouncer(ArmConfig.SHOULDER_HOMING_TIME_SECS.get());
                      homingDebouncer.calculate(false);
                    })
                .withName(name + "/Set Goal"),
            Commands.waitUntil(() -> this.atGoal() || (!this.atGoal() && hasStall.getAsBoolean()))
                .withName(name + "/Wait Until Goal"),
            Commands.startRun(
                    () -> isHoming = true,
                    () -> shoulderIO.setCurrent(ArmConfig.SHOULDER_HOMING_CURRENT_AMP.get()),
                    this)
                .until(hasStall)
                .withName(name + "/Set Current"),
            Commands.runOnce(() -> shoulderIO.resetAppliedPosition(0.0))
                .withName(name + "/Reset Applied Position"))
        .finallyDo(
            () -> {
              setGoal(ArmGoal.IDLE);
              isHoming = false;
            })
        .withName(name);
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
            UnitConverter.offset(ArmGoal.START.getShoulderHeightMeter()).withUnits("m", "m"),
            ArmConfig.SHOULDER_GAINS,
            0,
            ArmConfig.SHOULDER_MAX_HEIGHT_METER),
        new DCMotorIOSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), 0.46, ArmConfig.ELBOW_REDUCTION),
            DCMotor.getKrakenX60(1),
            UnitConverter.scale(180.0 / Math.PI).withUnits("rad", "deg"),
            UnitConverter.offset(ArmGoal.START.getElbowPositionDegree()).withUnits("deg", "deg"),
            ArmConfig.ELBOW_GAINS,
            -360,
            360));
  }

  public static Arm createReal() {
    return new Arm(
        new DCMotorIOTalonFX(
                "ArmShoulder",
                Constants.Ports.Can.ARM_SHOULDER_MASTER,
                ArmConfig.getShoulderTalonConfig(),
                UnitConverter.scale(ArmConfig.SHOULDER_METER_PER_ROTATION).withUnits("rot", "m"),
                UnitConverter.offset(ArmGoal.START.getShoulderHeightMeter()).withUnits("m", "m"))
            .withFollower(
                Constants.Ports.Can.ARM_SHOULDER_SLAVE, ArmConfig.getShoulderTalonConfig(), true),
        new DCMotorIOTalonFX(
                "ArmElbow",
                Constants.Ports.Can.ARM_ELBOW,
                ArmConfig.getElbowTalonConfig(),
                UnitConverter.scale(360).withUnits("rot", "deg"),
                UnitConverter.offset(360 * (-0.2958333 + 0.5)).withUnits("deg", "deg"))
            .withCancoder(
                "ArmElbow",
                Constants.Ports.Can.ARM_ELBOW_CANCODER,
                ArmConfig.getCancoderConfig(-0.2958333 + 0.5)));
  }

  public static Arm createIO() {
    return new Arm(new DCMotorIO() {}, new DCMotorIO() {});
  }

  public void registerTransform(TransformTree transformTree) {
    transformTree.registerTransformComponent(
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

    transformTree.registerTransformComponent(
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

    transformTree.registerTransformComponent(
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

    transformTree.registerTransformComponent(
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
