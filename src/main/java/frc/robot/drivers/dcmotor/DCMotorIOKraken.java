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

public class DCMotorIOKraken implements DCMotorIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;

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
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
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
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                acceleration,
                outputVoltage,
                supplyCurrent,
                statorCurrent,
                temperature)
            .isOK();

    inputs.rawPositionRad = position.getValueAsDouble();
    inputs.rawVelocityRadPerSec = velocity.getValueAsDouble();
    inputs.rawAccelerationRadPerSecSq = velocity.getValueAsDouble();

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

  public void setPositionF(
      double positionRad,
      double velocityRadPerSec,
      double accelerationRadPerSecSq,
      double feedforward) {
    motor.setControl(
        new DynamicMotionMagicTorqueCurrentFOC(
                positionRad, velocityRadPerSec, accelerationRadPerSecSq, 0.0)
            .withFeedForward(feedforward));
  }

  @Override
  public void setPosition(
      double positionRad, double velocityRadPerSec, double accelerationRadPerSecSq) {
    motor.setControl(new PositionTorqueCurrentFOC(positionRad).withVelocity(velocityRadPerSec));
  }

  @Override
  public void setPositionF(double positionRad, double feedforward) {
    motor.setControl(new PositionTorqueCurrentFOC(positionRad).withFeedForward(feedforward));
  }

  @Override
  public void setVelocityF(
      double velocityRadPerSec, double accelerationRadPerSecSq, double feedforward) {
    motor.setControl(
        new MotionMagicVelocityTorqueCurrentFOC(velocityRadPerSec)
            .withAcceleration(accelerationRadPerSecSq)
            .withFeedForward(feedforward));
  }

  @Override
  public void setVelocityF(double velocityRadPerSec, double feedforward) {
    motor.setControl(new VelocityTorqueCurrentFOC(velocityRadPerSec).withFeedForward(feedforward));
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

  public void withCancoder(int id) {
    config.Feedback.FeedbackRemoteSensorID = id;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
  }
}
