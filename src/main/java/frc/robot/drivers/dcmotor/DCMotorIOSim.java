package frc.robot.drivers.dcmotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Config;
import frc.robot.utils.GainsUtil.Gains;
import frc.robot.utils.UnitConverter;
import java.util.function.DoubleSupplier;

public class DCMotorIOSim implements DCMotorIO {
  private final DCMotorSim sim;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
  private final SlewRateLimiter voltageLimiter = new SlewRateLimiter(10);

  private UnitConverter ratioConverter = UnitConverter.identity();
  private UnitConverter positionConvertor = UnitConverter.identity();

  public DCMotorIOSim(
      DCMotor motor,
      double JKgMetersSquared,
      double gearing,
      UnitConverter ratioConverter,
      Gains gains) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, gearing), motor);

    setUnitConvertor(ratioConverter);
    setGains(gains);
  }

  public DCMotorIOSim(LinearSystem<N2, N1, N2> system, DCMotor motor) {
    sim = new DCMotorSim(system, motor);
  }

  @Override
  public void updateInputs(DCMotorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(Config.LOOP_PERIOD_SEC);

    inputs.connected = true;

    // rotor, radians
    inputs.rawPosition = sim.getAngularPositionRad();
    inputs.rawVelocity = sim.getAngularVelocityRadPerSec();
    inputs.rawAcceleration = sim.getAngularAccelerationRadPerSecSq();
    inputs.rawUnit = ratioConverter.getFromUnits();

    // mechanism, applied units
    inputs.appliedPosition = positionConvertor.applyAsDouble(inputs.rawPosition);
    inputs.appliedVelocity = positionConvertor.applyAsDouble(inputs.rawVelocity);
    inputs.appliedAcceleration = positionConvertor.applyAsDouble(inputs.rawAcceleration);
    inputs.appliedUnit = ratioConverter.getToUnits();

    inputs.outputVoltageVolts = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.temperatureCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setPidsg(double kp, double ki, double kd, double ks, double kg) {
    pid.setPID(kp, ki, kd);
  }

  @Override
  public void setPid(double kp, double ki, double kd) {
    pid.setPID(kp, ki, kd);
  }

  @Override
  public void setMotionConstraints(double maxVelocity, double maxAcceleration) {
    pid.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public void setRotationContinuous(boolean isContinuous) {
    if (isContinuous) {
      pid.enableContinuousInput(-Math.PI, Math.PI);
    } else {
      pid.disableContinuousInput();
    }
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
    double pidOutput =
        pid.calculate(sim.getAngularPositionRad(), positionConvertor.convertInverse(position));
    setVoltage(pidOutput + feedforward);
  }

  @Override
  public void setAppliedVelocityF(double velocity, double acceleration, double feedforward) {
    double pidOutput =
        pid.calculate(sim.getAngularVelocityRadPerSec(), ratioConverter.convertInverse(velocity));
    setVoltage(pidOutput + feedforward);
  }

  @Override
  public void setVoltage(double volts) {
    if (DriverStation.isEnabled()) {
      sim.setInputVoltage(voltageLimiter.calculate(volts));
    } else {
      voltageLimiter.reset(0);
      sim.setInputVoltage(0);
    }
  }

  public void setVoltage(DoubleSupplier volts) {
    setVoltage(volts.getAsDouble());
  }

  @Override
  public void setCurrent(double amps) {
    double resistance = 0.1; // Simulated motor resistance
    double kv = 0.01; // Simulated back-EMF constant
    double voltage = amps * resistance + sim.getAngularVelocityRadPerSec() * kv;
    setVoltage(voltage);
  }

  @Override
  public void resetPosition(double position) {
    sim.setState(position, sim.getAngularVelocityRadPerSec());
  }

  @Override
  public void follow(DCMotorIO motor, Boolean isInverted) {
    setVoltage(() -> (isInverted ? -motor.getVoltage() : motor.getVoltage()));
  }

  @Override
  public double getVoltage() {
    return sim.getInputVoltage();
  }

  @Override
  public double getCurrent() {
    return sim.getCurrentDrawAmps();
  }

  @Override
  public double getAppliedPosition() {
    return positionConvertor.applyAsDouble(sim.getAngularPositionRad());
  }

  @Override
  public double getAppliedVelocity() {
    return ratioConverter.applyAsDouble(sim.getAngularVelocityRadPerSec());
  }

  @Override
  public double getAppliedAcceleration() {
    return ratioConverter.applyAsDouble(sim.getAngularAccelerationRadPerSecSq());
  }

  @Override
  public int getDeviceID() {
    return 0; // Simulation doesn't have device IDs
  }
}
