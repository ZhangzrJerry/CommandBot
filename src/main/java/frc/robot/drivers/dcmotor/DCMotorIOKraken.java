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
import lombok.Getter;

public class DCMotorIOKraken implements DCMotorIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;

  private UnitConverter ratioConverter = UnitConverter.identity();
  private UnitConverter positionConvertor = UnitConverter.identity();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> outputVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  public DCMotorIOKraken(String name, CanDevice device, TalonFXConfiguration config) {
    this.motor = new TalonFX(device.id(), device.bus());
    this.config = config;

    var wrappedName = "[" + name + "]";
    PhoenixHelper.checkErrorAndRetry(wrappedName + " clear sticky fault", motor::clearStickyFaults);
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " config", () -> motor.getConfigurator().apply(config));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    outputVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temperature = motor.getDeviceTemp();

    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () -> BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            position,
            velocity,
            acceleration,
            outputVoltage,
            supplyCurrent,
            statorCurrent,
            temperature));
    PhoenixHelper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", motor::optimizeBusUtilization);
  }

  public void updateInputs(DCMotorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(
        position,
        velocity,
        acceleration,
        outputVoltage,
        supplyCurrent,
        statorCurrent,
        temperature)
        .isOK();

    // mechanism, rotated units
    inputs.angularPosition = position.getValueAsDouble();
    inputs.angularVelocity = velocity.getValueAsDouble();
    inputs.angularAcceleration = velocity.getValueAsDouble();

    // mechanism, applied units
    inputs.position = positionConvertor.applyAsDouble(inputs.angularPosition);
    inputs.velocity = ratioConverter.applyAsDouble(inputs.angularVelocity);
    inputs.acceleration = ratioConverter.applyAsDouble(inputs.angularAcceleration);

    inputs.outputVoltageVolts = outputVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temperature.getValueAsDouble();
  }

  public void setPIDF(double kp, double ki, double kd, double ks, double kg) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    config.Slot0.kS = ks;
    config.Slot0.kG = kg;
    motor.getConfigurator().apply(config);
  }

  public void setPID(double kp, double ki, double kd) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    motor.getConfigurator().apply(config);
  }

  public void setMotionConstraints(double maxVelocityRadPerSec, double maxAccelerationRadPerSecSq) {
    // TODO
  }

  public void setUnitConvertor(UnitConverter ratioConverter, UnitConverter... offsetConverter) {
    this.ratioConverter = ratioConverter;
    if (offsetConverter.length > 0) {
      this.positionConvertor = ratioConverter.andThen(offsetConverter[0]);
    } else {
      this.positionConvertor = ratioConverter;
    }
  }

  public void setPositionF(
      double position,
      double velocity,
      double acceleration,
      double feedforward) {
    motor.setControl(
        new DynamicMotionMagicTorqueCurrentFOC(
            ratioConverter.convertInverse(position),
            ratioConverter.convertInverse(velocity),
            ratioConverter.convertInverse(acceleration),
            0.0)
            .withFeedForward(feedforward));
  }

  @Override
  public void setPosition(
      double position, double velocity, double acceleration) {
    motor.setControl(
        new PositionTorqueCurrentFOC(ratioConverter.convertInverse(position))
            .withVelocity(ratioConverter.convertInverse(velocity)));
  }

  @Override
  public void setPositionF(double position, double feedforward) {
    motor.setControl(
        new PositionTorqueCurrentFOC(ratioConverter.convertInverse(position))
            .withFeedForward(feedforward));
  }

  @Override
  public void setVelocityF(
      double velocity, double acceleration, double feedforward) {
    motor.setControl(
        new MotionMagicVelocityTorqueCurrentFOC(ratioConverter.convertInverse(velocity))
            .withAcceleration(ratioConverter.convertInverse(acceleration))
            .withFeedForward(feedforward));
  }

  @Override
  public void setVelocityF(double velocity, double feedforward) {
    motor.setControl(
        new VelocityTorqueCurrentFOC(ratioConverter.convertInverse(velocity))
            .withFeedForward(feedforward));
  }

  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  public void setCurrent(double amps) {
    motor.setControl(new TorqueCurrentFOC(amps));
  }

  public void resetPosition(double positionRad) {
    motor.set(positionRad);
  }

  public void follow(DCMotorIO motor, Boolean isInverted) {
    this.motor.setControl(new Follower(motor.getDeviceID(), isInverted));
  }

  public double getVoltage() {
    return outputVoltage.getValueAsDouble();
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
