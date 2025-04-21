package frc.robot.drivers.dcmotor;

import org.littletonrobotics.junction.AutoLog;

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
   * Sets PIDF gains for the controller
   *
   * @param kp Proportional gain
   * @param ki Integral gain (optional, default 0)
   * @param kd Derivative gain (optional, default 0)
   * @param ks
   * @param kg
   */
  void setPIDF(double kp, double ki, double kd, double ks, double kg);

  /**
   * Sets PIDF gains for the controller
   *
   * @param kp Proportional gain
   * @param ki Integral gain (optional, default 0)
   * @param kd Derivative gain (optional, default 0)
   */
  void setPID(double kp, double ki, double kd);

  /**
   * Sets motion profile constraints
   *
   * @param maxVelocityRadPerSec       Maximum velocity in rad/s
   * @param maxAccelerationRadPerSecSq Maximum acceleration in rad/s²
   */
  void setMotionConstraints(double maxVelocityRadPerSec, double maxAccelerationRadPerSecSq);

  // ========== Control Methods ==========

  /**
   * Sets target position with velocity, acceleration and feedforwrd
   *
   * @param positionRad             Target position in radians
   * @param velocityRadPerSec       Velocity in rad/s
   * @param accelerationRadPerSecSq Acceleration in rad/s²
   * @param feedforward
   */
  void setPositionF(
      double positionRad,
      double velocityRadPerSec,
      double accelerationRadPerSecSq,
      double feedforward);

  default void setPosition(
      double positionRad,
      double velocityRadPerSec,
      double accelerationRadPerSecSq) {
    setPositionF(positionRad, velocityRadPerSec, accelerationRadPerSecSq, 0.0);
  }

  /**
   * Sets target position with feedforward
   *
   * @param positionRad Target position in radians
   * @param feedforward
   */
  default void setPositionF(double positionRad, double feedforward) {
    setPositionF(positionRad, 0, 0, feedforward);
  }

  /**
   * Sets target position without feedforward
   *
   * @param positionRad Target position in radians
   */
  default void setPosition(double positionRad) {
    setPositionF(positionRad, 0);
  }

  /**
   * Sets target velocity
   *
   * @param velocityRadPerSec       Target velocity in rad/s
   * @param accelerationRadPerSecSq Acceleration in rad/s²
   * @param feedforward
   */
  void setVelocityF(double velocityRadPerSec, double accelerationRadPerSecSq, double feedforward);

  default void setVelocity(double velocityRadPerSec, double accelerationRadPerSecSq) {
    setVelocityF(velocityRadPerSec, accelerationRadPerSecSq, 0.0);
  }

  /** */
  default void setVelocityF(double velocityRadPerSec, double feedforward) {
    setVelocityF(velocityRadPerSec, 0, feedforward);
  }

  /**
   * Sets target velocity
   *
   * @param velocityRadPerSec Target velocity in rad/s
   */
  default void setVelocity(double velocityRadPerSec) {
    setVelocityF(velocityRadPerSec, 0);
  }

  /**
   * Sets voltage output
   *
   * @param volts Voltage to apply (-12 to 12V)
   */
  void setVoltage(double volts);

  /**
   * Sets current target
   *
   * @param amps Current in amps
   */
  void setCurrent(double amps);

  /** Stops the motor (sets voltage to 0) */
  default void stop() {
    setVoltage(0.0);
  }

  /**
   * Resets the motor position
   *
   * @param position New position in radians
   */
  void resetPosition(double position);

  /**
   * Follow a motor
   *
   * @param motor
   */
  void follow(DCMotorIO motor, Boolean isInverted);

  // ========== Status Methods ==========

  /**
   * Updates the inputs with current sensor data
   *
   * @param inputs Inputs object to populate
   */
  void updateInputs(DCMotorIOInputs inputs);

  double getVoltage();

  default int getDeviceID() {
    return 0;
  }
}
