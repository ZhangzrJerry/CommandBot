package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfx;
import frc.robot.services.VisualizeService;
import frc.robot.utils.Gains.GainsImpl;
import frc.robot.utils.dashboard.AlertManager;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.UnitConverter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  @RequiredArgsConstructor
  public enum IntakeGoal {
    IDLE(() -> 0.0, IntakeConfig.slamUpCurrentAmp),
    IIDLE(() -> 0.0, IntakeConfig.slamUpCurrentAmp),
    DODGE(() -> 0.0, IntakeConfig.slamDownCurrentAmp),
    COLLECT(
        new TunableNumber("Intake/Roller/Goal/CollectVoltageVolt", -5.5),
        IntakeConfig.slamDownCurrentAmp),
    EJECT(
        new TunableNumber("Intake/Roller/Goal/EjectVoltageVolt", 5.5),
        IntakeConfig.slamDownCurrentAmp);

    private final DoubleSupplier rollerVoltageVoltSupplier;
    private final DoubleSupplier slamCurrentAmpSupplier;

    double getRollerVoltageVolt() {
      return rollerVoltageVoltSupplier.getAsDouble();
    }

    double getSlamCurrentAmp() {
      return slamCurrentAmpSupplier.getAsDouble();
    }
  }

  private final DCMotorIO rollerIO;
  private final DCMotorIO armIO;

  private final DCMotorIOInputsAutoLogged rollerIOInputs = new DCMotorIOInputsAutoLogged();
  private final DCMotorIOInputsAutoLogged armIOInputs = new DCMotorIOInputsAutoLogged();

  private final AlertManager rollerOfflineAlert =
      new AlertManager("Algae ground intake roller motor offline!", AlertManager.AlertType.WARNING);
  private final AlertManager armOfflineAlert =
      new AlertManager("Algae ground intake arm motor offline!", AlertManager.AlertType.WARNING);

  @Getter @AutoLogOutput private IntakeGoal goal = IntakeGoal.IDLE;

  private boolean slammed = false;
  private Debouncer slamDebouncer = new Debouncer(0.0);

  @Setter private BooleanSupplier dodgeSignalSupplier = () -> false;
  private final Debouncer needDodgeDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

  @Override
  public void periodic() {
    updateInputs();

    rollerOfflineAlert.set(!rollerIOInputs.connected);
    armOfflineAlert.set(!armIOInputs.connected);

    var needDodge = needDodgeDebouncer.calculate(dodgeSignalSupplier.getAsBoolean());
    if (goal == IntakeGoal.IDLE && needDodge) {
      setGoal(IntakeGoal.DODGE);
    }

    if (goal == IntakeGoal.DODGE && !needDodge) {
      setGoal(IntakeGoal.IDLE);
    }

    rollerIO.setVoltage(goal.getRollerVoltageVolt());

    if (slamDebouncer.calculate(
        Math.abs(armIOInputs.appliedVelocity)
            <= Units.degreesToRadians(IntakeConfig.slamVelocityThreshDegreePerSec.getAsDouble()))) {
      slammed = true;
    }

    if (!slammed) {
      armIO.setCurrent(goal.getSlamCurrentAmp());
    } else {
      armIO.stop();
    }

    if (DriverStation.isDisabled()) {
      slammed = false;
      slamDebouncer.calculate(false);
    }
  }

  public void setGoal(IntakeGoal goal) {
    if (this.goal != goal) {
      slammed = false;
      slamDebouncer = new Debouncer(IntakeConfig.slamTimeSecs.getAsDouble());
      slamDebouncer.calculate(false);
      this.goal = goal;
    }
  }

  @AutoLogOutput(key = "Intake/AtGoal")
  public boolean atGoal() {
    return slammed;
  }

  private void updateInputs() {
    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Roller", rollerIOInputs);

    armIO.updateInputs(armIOInputs);
    Logger.processInputs("Intake/Arm", armIOInputs);
  }

  public void stop() {
    rollerIO.stop();
    armIO.stop();
  }

  private Intake(DCMotorIO rollerIO, DCMotorIO armIO) {
    this.rollerIO = rollerIO;
    this.armIO = armIO;
  }

  public static Intake createReal() {
    return new Intake(
        new DCMotorIOTalonfx(
            "IntakeRoller",
            Constants.Ports.Can.INTAKE_ROLLER,
            IntakeConfig.getRollerTalonConfig(),
            UnitConverter.scale(2 * Math.PI).withUnits("rot", "rad")),
        new DCMotorIOTalonfx(
            "IntakeArm",
            Constants.Ports.Can.INTAKE_ARM,
            IntakeConfig.getArmTalonConfig(),
            UnitConverter.scale(2 * Math.PI).withUnits("rot", "rad")));
  }

  public static Intake createSim() {
    return new Intake(
        new DCMotorIOSim(
            DCMotor.getKrakenX60Foc(1),
            1.0,
            1.0,
            UnitConverter.identity().withUnits("rad", "rad"),
            new GainsImpl(1.0, 0.0, 0.0, 0.0, 0.0)),
        new DCMotorIOSim(
            DCMotor.getKrakenX60Foc(1),
            1.0,
            1.0,
            UnitConverter.identity().withUnits("rad", "rad"),
            new GainsImpl(1.0, 0.0, 0.0, 0.0, 0.0)));
  }

  public static Intake createIO() {
    return new Intake(new DCMotorIO() {}, new DCMotorIO() {});
  }

  public void registerVisualize(VisualizeService visualizer) {
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.INTAKE,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            new Transform3d(
                0,
                0,
                0,
                new Rotation3d(
                    (Units.degreesToRadians(IntakeConfig.ARM_HOME_POSITION_DEGREE)
                        - (armIOInputs.appliedPosition
                            - Units.degreesToRadians(IntakeConfig.ARM_HOME_POSITION_DEGREE))),
                    0,
                    0)));
  }
}
