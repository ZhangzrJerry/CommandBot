package frc.robot.drivers.dcmotor;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.UnitConverter;

public interface DCMotorIO {
  @AutoLog
  class DCMotorIOInputs {
    // Connection status
    public boolean connected = false;

    // Raw sensor data (radians)
    public double rawPositionRad = 0.0;
    public double rawVelocityRadPerSec = 0.0;
    public double rawAccelerationRadPerSecSq = 0.0;

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
   * @param maxVelocityRadPerSec       Maximum velocity in radians per second
   * @param maxAccelerationRadPerSecSq Maximum acceleration in radians per second
   *                                   squared
   */
  void setMotionConstraints(double maxVelocityRadPerSec, double maxAccelerationRadPerSecSq);

  void setUnitConvertor(UnitConverter ratioConverter, UnitConverter... offsetConverter);

  // ========== Control Methods ==========

  /**
   * Sets target position with full motion control parameters.
   *
   * @param positionRad             Target position in radians
   * @param velocityRadPerSec       Target velocity in radians per second
   * @param accelerationRadPerSecSq Target acceleration in radians per second
   *                                squared
   * @param feedforward             Additional feedforward voltage
   */
  void setPositionF(
      double positionRad,
      double velocityRadPerSec,
      double accelerationRadPerSecSq,
      double feedforward);

  /**
   * Sets target position with velocity and acceleration parameters.
   *
   * @param positionRad             Target position in radians
   * @param velocityRadPerSec       Target velocity in radians per second
   * @param accelerationRadPerSecSq Target acceleration in radians per second
   *                                squared
   */
  default void setPosition(
      double positionRad, double velocityRadPerSec, double accelerationRadPerSecSq) {
    setPositionF(positionRad, velocityRadPerSec, accelerationRadPerSecSq, 0.0);
  }

  /**
   * Sets target position with feedforward voltage.
   *
   * @param positionRad Target position in radians
   * @param feedforward Additional feedforward voltage
   */
  default void setPositionF(double positionRad, double feedforward) {
    setPositionF(positionRad, 0, 0, feedforward);
  }

  /**
   * Sets target position with basic control.
   *
   * @param positionRad Target position in radians
   */
  default void setPosition(double positionRad) {
    setPositionF(positionRad, 0);
  }

  /**
   * Sets target velocity with full motion control parameters.
   *
   * @param velocityRadPerSec       Target velocity in radians per second
   * @param accelerationRadPerSecSq Target acceleration in radians per second
   *                                squared
   * @param feedforward             Additional feedforward voltage
   */
  void setVelocityF(double velocityRadPerSec, double accelerationRadPerSecSq, double feedforward);

  /**
   * Sets target velocity with acceleration control.
   *
   * @param velocityRadPerSec       Target velocity in radians per second
   * @param accelerationRadPerSecSq Target acceleration in radians per second
   *                                squared
   */
  default void setVelocity(double velocityRadPerSec, double accelerationRadPerSecSq) {
    setVelocityF(velocityRadPerSec, accelerationRadPerSecSq, 0.0);
  }

  /**
   * Sets target velocity with feedforward voltage.
   *
   * @param velocityRadPerSec Target velocity in radians per second
   * @param feedforward       Additional feedforward voltage
   */
  default void setVelocityF(double velocityRadPerSec, double feedforward) {
    setVelocityF(velocityRadPerSec, 0, feedforward);
  }

  /**
   * Sets target velocity with basic control.
   *
   * @param velocityRadPerSec Target velocity in radians per second
   */
  default void setVelocity(double velocityRadPerSec) {
    setVelocityF(velocityRadPerSec, 0);
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
