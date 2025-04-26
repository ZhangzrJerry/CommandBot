package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfx;
import frc.robot.interfaces.hardwares.sensors.digital.BiDigitalIO;
import frc.robot.interfaces.hardwares.sensors.digital.BiDigitalIOCandi;
import frc.robot.interfaces.hardwares.sensors.digital.BiDigitalIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.sensors.digital.BiDigitalIOSim;
import frc.robot.services.TransformTree;
import frc.robot.utils.dashboard.SwitchableChooser;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.UnitConverter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Endeffector extends SubsystemBase {
  @RequiredArgsConstructor
  public enum AlgaeEndEffectorGoal {
    IDLE(new TunableNumber("EndEffector/Algae/Goal/IdleVoltageVolt", 0.0)),
    HOLDING(new TunableNumber("EndEffector/Algae/Goal/HoldingVoltageVolt", -3.25)),
    COLLECT(new TunableNumber("EndEffector/Algae/Goal/CollectVoltageVolt", -4.5)),
    EJECT(new TunableNumber("EndEffector/Algae/Goal/EjectVoltageVolt", 3.0)),
    SCORE(new TunableNumber("EndEffector/Algae/Goal/ScoreVoltageVolt", 2.0));

    private final DoubleSupplier voltage;

    double getVoltageVolt() {
      return voltage.getAsDouble();
    }
  }

  @RequiredArgsConstructor
  public enum CoralEndEffectorGoal {
    IDLE(new TunableNumber("EndEffector/Coral/Goal/IdleVoltageVolt", 0.0)),
    HOLDING(new TunableNumber("EndEffector/Coral/Goal/HoldingVoltageVolt", -1.25)),
    COLLECT(new TunableNumber("EndEffector/Coral/Goal/CollectVoltageVolt", -3.0)),
    EJECT(new TunableNumber("EndEffector/Coral/Goal/EjectVoltageVolt", 3.0)),
    SCORE(new TunableNumber("EndEffector/Coral/Goal/ScoreVoltageVolt", 6.0));

    private final DoubleSupplier voltage;

    double getVoltageVolt() {
      return voltage.getAsDouble();
    }
  }

  @Setter BooleanSupplier algaeSignalSupplier = () -> false;
  @Setter BooleanSupplier coralSignalSupplier = () -> false;

  @AutoLogOutput(key = "Endeffector/Algae Sensor Reliable")
  boolean algaeSensorReliable = true;

  @AutoLogOutput(key = "Endeffector/Coral Sensor Reliable")
  boolean coralSensorReliable = true;

  SwitchableChooser algaeSensorReliableChooser =
      new SwitchableChooser("Endeffector/Algae Sensor Reliable");
  SwitchableChooser coralSensorReliableChooser =
      new SwitchableChooser("Endeffector/Coral Sensor Reliable");

  private final DCMotorIO algaeIO;
  private final DCMotorIO coralIO;
  private final BiDigitalIO digitalIO;

  private final DCMotorIOInputsAutoLogged algaeIOInputs = new DCMotorIOInputsAutoLogged();
  private final DCMotorIOInputsAutoLogged coralIOInputs = new DCMotorIOInputsAutoLogged();
  private final BiDigitalIOInputsAutoLogged digitalIOInputs = new BiDigitalIOInputsAutoLogged();

  @Setter @Getter private AlgaeEndEffectorGoal algaeGoal = AlgaeEndEffectorGoal.IDLE;
  @Setter @Getter private CoralEndEffectorGoal coralGoal = CoralEndEffectorGoal.IDLE;

  @Override
  public void periodic() {
    updateInputs();

    if (algaeGoal != AlgaeEndEffectorGoal.SCORE
        && algaeGoal != AlgaeEndEffectorGoal.EJECT
        && hasAlgaeEndeffectorStoraged()) {
      setAlgaeGoal(AlgaeEndEffectorGoal.HOLDING);
    }
    algaeIO.setVoltage(algaeGoal.getVoltageVolt());

    if (coralGoal != CoralEndEffectorGoal.SCORE
        && coralGoal != CoralEndEffectorGoal.EJECT
        && hasCoralEndeffectorStoraged()) {
      setCoralGoal(CoralEndEffectorGoal.HOLDING);
    }
    coralIO.setVoltage(coralGoal.getVoltageVolt());
  }

  public void updateInputs() {
    algaeIO.updateInputs(algaeIOInputs);
    Logger.processInputs("Endeffector/Algae", algaeIOInputs);

    coralIO.updateInputs(coralIOInputs);
    Logger.processInputs("Endeffector/Coral", coralIOInputs);

    digitalIO.updateInputs(digitalIOInputs);
    Logger.processInputs("Endeffector/DigitalIO", digitalIOInputs);

    algaeSensorReliable = algaeSensorReliableChooser.get().equals("True");
    coralSensorReliable = coralSensorReliableChooser.get().equals("True");
  }

  @AutoLogOutput(key = "Endeffector/Algae EndEffector Storaged")
  public boolean hasAlgaeEndeffectorStoraged() {
    return (algaeSensorReliable && digitalIO.getValue1())
        || (!algaeSensorReliable && algaeSignalSupplier.getAsBoolean());
  }

  @AutoLogOutput(key = "Endeffector/Coral EndEffector Storaged")
  public boolean hasCoralEndeffectorStoraged() {
    return (coralSensorReliable && digitalIO.getValue2())
        || (!coralSensorReliable && coralSignalSupplier.getAsBoolean());
  }

  private Endeffector(DCMotorIO algaeIO, DCMotorIO coralIO, BiDigitalIO digitalIO) {
    this.algaeIO = algaeIO;
    this.coralIO = coralIO;
    this.digitalIO = digitalIO;
    algaeSensorReliableChooser.setOptions(new String[] {"True", "False"});
    coralSensorReliableChooser.setOptions(new String[] {"True", "False"});
  }

  public static Endeffector createReal() {
    return new Endeffector(
        new DCMotorIOTalonfx(
            "Algae EndEffector",
            Constants.Ports.Can.ALGAE_END_EFFECTOR,
            EndeffectorConfig.getAlgaeTalonConfig(),
            UnitConverter.identity()),
        new DCMotorIOTalonfx(
            "Coral EndEffector",
            Constants.Ports.Can.CORAL_END_EFFECTOR,
            EndeffectorConfig.getCoralTalonConfig(),
            UnitConverter.identity()),
        new BiDigitalIOCandi(
            "Arm Candi", Constants.Ports.Can.ARM_CANDI, EndeffectorConfig.getCandiConfig()));
  }

  public static Endeffector createSim(
      BooleanSupplier algaeSignalSupplier, BooleanSupplier coralSignalSupplier) {
    return new Endeffector(
        new DCMotorIOSim(),
        new DCMotorIOSim(),
        new BiDigitalIOSim(algaeSignalSupplier, coralSignalSupplier));
  }

  public static Endeffector createIO() {
    return new Endeffector(new DCMotorIO() {}, new DCMotorIO() {}, new BiDigitalIO() {});
  }

  public void registerTransform(TransformTree transformTree) {
    transformTree.registerTransformComponent(
        Constants.Ascope.Component.ALGAE_END_EFFECTOR,
        Constants.Ascope.Component.ARM,
        () -> new Transform3d(.04, -.46, .07, new Rotation3d()));

    transformTree.registerTransformComponent(
        Constants.Ascope.Component.CORAL_END_EFFECTOR,
        Constants.Ascope.Component.ARM,
        () -> new Transform3d(-.05, .25, .0, new Rotation3d()));

    transformTree.registerTransformComponent(
        Constants.Ascope.Component.ALGAE,
        Constants.Ascope.Component.ARM,
        () ->
            hasAlgaeEndeffectorStoraged()
                ? new Transform3d(.04, -.46, .07, new Rotation3d())
                : new Transform3d(0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f, new Rotation3d()));

    transformTree.registerTransformComponent(
        Constants.Ascope.Component.CORAL,
        Constants.Ascope.Component.ARM,
        () ->
            hasCoralEndeffectorStoraged()
                ? new Transform3d(-.05, .25, .0, new Rotation3d(0, 0, Math.PI / 2))
                : new Transform3d(0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f, new Rotation3d()));
  }
}
