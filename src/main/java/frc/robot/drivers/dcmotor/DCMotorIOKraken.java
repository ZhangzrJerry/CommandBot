package frc.robot.drivers.dcmotor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.CanDevice;
import frc.robot.utils.PhoenixHelper;
import frc.robot.utils.UnitConverter;

public class DCMotorIOKraken implements DCMotorIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;

  private UnitConverter ratioConverter = UnitConverter.identity();
  private UnitConverter positionConvertor = UnitConverter.identity();

  private final StatusSignal<Angle> rawPosition;
  private final StatusSignal<AngularVelocity> rawVelocity;
  private final StatusSignal<AngularAcceleration> rawAcceleration;
  private final StatusSignal<Voltage> outputVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  public DCMotorIOKraken(
      String name,
      CanDevice kraken,
      TalonFXConfiguration config,
      UnitConverter ratioConverter,
      UnitConverter... offsetConverter) {
    this.motor = new TalonFX(kraken.id(), kraken.bus());
    this.config = config;
    setUnitConvertor(ratioConverter, offsetConverter);

    var wrappedName = "[" + name + "]";
    PhoenixHelper.checkErrorAndRetry(wrappedName + " clear sticky fault", motor::clearStickyFaults);
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " config", () -> motor.getConfigurator().apply(config));

    rawPosition = motor.getPosition();
    rawVelocity = motor.getVelocity();
    rawAcceleration = motor.getAcceleration();
    outputVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temperature = motor.getDeviceTemp();

    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () -> BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            rawPosition,
            rawVelocity,
            rawAcceleration,
            outputVoltage,
            supplyCurrent,
            statorCurrent,
            temperature));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", motor::optimizeBusUtilization);
  }

  public void updateInputs(DCMotorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(
        rawPosition,
        rawVelocity,
        rawAcceleration,
        outputVoltage,
        supplyCurrent,
        statorCurrent,
        temperature)
        .isOK();

    // mechanism, rotated units
    inputs.rawPosition = rawPosition.getValueAsDouble();
    inputs.rawVelocity = rawVelocity.getValueAsDouble();
    inputs.rawAcceleration = rawVelocity.getValueAsDouble();
    inputs.rawUnit = ratioConverter.getFromUnits();

    // mechanism, applied units
    inputs.appliedPosition = positionConvertor.applyAsDouble(inputs.rawPosition);
    inputs.appliedVelocity = ratioConverter.applyAsDouble(inputs.rawVelocity);
    inputs.appliedAcceleration = ratioConverter.applyAsDouble(inputs.rawAcceleration);
    inputs.appliedUnit = ratioConverter.getToUnits();

    inputs.outputVoltageVolts = outputVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setPIDF(double kp, double ki, double kd, double ks, double kg) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    config.Slot0.kS = ks;
    config.Slot0.kG = kg;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setMotionConstraints(double maxVelocityRadPerSec, double maxAccelerationRadPerSecSq) {
    // TODO
  }

  @Override
  public void setRotationContinuous(boolean isContinuous) {
    config.ClosedLoopGeneral.ContinuousWrap = isContinuous;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setUnitConvertor(UnitConverter ratioConverter, UnitConverter... offsetConverter) {
    this.ratioConverter = ratioConverter;
    if (offsetConverter.length > 0) {
      this.positionConvertor = offsetConverter[0].andThen(ratioConverter);
    } else {
      this.positionConvertor = ratioConverter;
    }
  }

  @Override
  public void setAppliedPositionF(
      double position, double velocity, double acceleration, double feedforward) {
    motor.setControl(
        new DynamicMotionMagicTorqueCurrentFOC(
            ratioConverter.convertInverse(position),
            ratioConverter.convertInverse(velocity),
            ratioConverter.convertInverse(acceleration),
            0.0)
            .withFeedForward(feedforward));
  }

  @Override
  public void setAppliedPosition(double position, double velocity, double acceleration) {
    motor.setControl(
        new PositionTorqueCurrentFOC(ratioConverter.convertInverse(position))
            .withVelocity(ratioConverter.convertInverse(velocity)));
  }

  @Override
  public void setAppliedPositionF(double position, double feedforward) {
    motor.setControl(
        new PositionTorqueCurrentFOC(ratioConverter.convertInverse(position))
            .withFeedForward(feedforward));
  }

  @Override
  public void setAppliedVelocityF(double velocity, double acceleration, double feedforward) {
    motor.setControl(
        new MotionMagicVelocityTorqueCurrentFOC(ratioConverter.convertInverse(velocity))
            .withAcceleration(ratioConverter.convertInverse(acceleration))
            .withFeedForward(feedforward));
  }

  @Override
  public void setAppliedVelocityF(double velocity, double feedforward) {
    motor.setControl(
        new VelocityTorqueCurrentFOC(ratioConverter.convertInverse(velocity))
            .withFeedForward(feedforward));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setCurrent(double amps) {
    motor.setControl(new TorqueCurrentFOC(amps));
  }

  @Override
  public void resetPosition(double positionRad) {
    motor.set(positionRad);
  }

  @Override
  public void follow(DCMotorIO motor, Boolean isInverted) {
    this.motor.setControl(new Follower(motor.getDeviceID(), isInverted));
  }

  @Override
  public double getVoltage() {
    return outputVoltage.getValueAsDouble();
  }

  @Override
  public double getCurrent() {
    return supplyCurrent.getValueAsDouble();
  }

  @Override
  public double getAppliedPosition() {
    return positionConvertor.applyAsDouble(rawPosition.getValueAsDouble());
  }

  @Override
  public double getAppliedVelocity() {
    return ratioConverter.applyAsDouble(rawVelocity.getValueAsDouble());
  }

  @Override
  public double getAppliedAcceleration() {
    return ratioConverter.applyAsDouble(rawAcceleration.getValueAsDouble());
  }

  @Override
  public int getDeviceID() {
    return motor.getDeviceID();
  }

  public void withCancoder(String name, int id) {
    var wrappedName = "[" + name + "]";
    config.Feedback.FeedbackRemoteSensorID = id;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " config fused cancoder mode", () -> motor.getConfigurator().apply(config));
  }
}
