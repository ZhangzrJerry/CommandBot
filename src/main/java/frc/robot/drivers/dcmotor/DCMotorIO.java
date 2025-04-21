package frc.robot.drivers.dcmotor;

import frc.robot.utils.UnitConverter;
import org.littletonrobotics.junction.AutoLog;

public interface DCMotorIO {
  @AutoLog
  class DCMotorIOInputs {
    // Connection status
    public boolean connected = false;

    // Raw sensor data (angular units)
    public double angularPosition = 0.0;
    public double angularVelocity = 0.0;
    public double angularAcceleration = 0.0;

    // Mechanism data (applied units)
    public double position = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;

    // Electrical measurements
    public double outputVoltageVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  // ========== Configuration Methods ==========

  /**
   * Configures PIDF gains for the motor controller.
   *
   * @param kp Proportional gain
   * @param ki Integral gain (optional, default 0)
   * @param kd Derivative gain (optional, default 0)
   * @param ks Static feedforward gain
   * @param kg Gravity feedforward gain
   */
  void setPIDF(double kp, double ki, double kd, double ks, double kg);

  /**
   * Configures PID gains for the motor controller.
   *
   * @param kp Proportional gain
   * @param ki Integral gain (optional, default 0)
   * @param kd Derivative gain (optional, default 0)
   */
  void setPID(double kp, double ki, double kd);

  /**
   * Sets motion profile constraints for smart motion control.
   *
   * @param maxVelocity     Maximum velocity in radians per second
   * @param maxAcceleration Maximum acceleration in radians per second squared
   */
  void setMotionConstraints(double maxVelocity, double maxAcceleration);

  /**
   * Sets the unit conversion for position and velocity.
   *
   * @param ratioConverter  Converter for position, velocity, and acceleration
   * @param offsetConverter Converter for position (optional)
   */
  void setUnitConvertor(UnitConverter ratioConverter, UnitConverter... offsetConverter);

  // ========== Control Methods ==========

  /**
   * Sets target position with full motion control parameters.
   *
   * @param position     Target position in radians
   * @param velocity     Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   * @param feedforward  Additional feedforward voltage
   */
  void setPositionF(double position, double velocity, double acceleration, double feedforward);

  /**
   * Sets target position with velocity and acceleration parameters.
   *
   * @param position     Target position in radians
   * @param velocity     Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   */
  default void setPosition(double position, double velocity, double acceleration) {
    setPositionF(position, velocity, acceleration, 0.0);
  }

  /**
   * Sets target position with feedforward voltage.
   *
   * @param position    Target position in radians
   * @param feedforward Additional feedforward voltage
   */
  default void setPositionF(double position, double feedforward) {
    setPositionF(position, 0, 0, feedforward);
  }

  /**
   * Sets target position with basic control.
   *
   * @param position Target position in radians
   */
  default void setPosition(double position) {
    setPositionF(position, 0);
  }

  /**
   * Sets target velocity with full motion control parameters.
   *
   * @param velocity     Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   * @param feedforward  Additional feedforward voltage
   */
  void setVelocityF(double velocity, double acceleration, double feedforward);

  /**
   * Sets target velocity with acceleration control.
   *
   * @param velocity     Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   */
  default void setVelocity(double velocity, double acceleration) {
    setVelocityF(velocity, acceleration, 0.0);
  }

  /**
   * Sets target velocity with feedforward voltage.
   *
   * @param velocity    Target velocity in radians per second
   * @param feedforward Additional feedforward voltage
   */
  default void setVelocityF(double velocity, double feedforward) {
    setVelocityF(velocity, 0, feedforward);
  }

  /**
   * Sets target velocity with basic control.
   *
   * @param velocity Target velocity in radians per second
   */
  default void setVelocity(double velocity) {
    setVelocityF(velocity, 0);
  }

  /**
   * Sets direct voltage output to motor.
   *
   * @param volts Voltage to apply (-12 to 12V)
   */
  void setVoltage(double volts);

  /**
   * Sets current limit for the motor.
   *
   * @param amps Current limit in amps
   */
  void setCurrent(double amps);

  /** Stops the motor by setting voltage to 0. */
  default void stop() {
    setVoltage(0.0);
  }

  /**
   * Resets the motor position sensor.
   *
   * @param position New position in radians
   */
  void resetPosition(double position);

  /**
   * Configures this motor to follow another motor.
   *
   * @param motor      The motor to follow
   * @param isInverted Whether to follow inverted
   */
  void follow(DCMotorIO motor, Boolean isInverted);

  // ========== Status Methods ==========

  /**
   * Updates the provided inputs object with current sensor data.
   *
   * @param inputs Inputs object to populate with current data
   */
  void updateInputs(DCMotorIOInputs inputs);

  /**
   * Gets the current output voltage of the motor.
   *
   * @return Current output voltage in volts
   */
  double getVoltage();

  /**
   * Gets the device ID of the motor controller.
   *
   * @return Device ID (default implementation returns 0)
   */
  default int getDeviceID() {
    return 0;
  }
}
