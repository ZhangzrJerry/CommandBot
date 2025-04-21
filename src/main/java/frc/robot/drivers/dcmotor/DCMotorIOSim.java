package frc.robot.drivers.dcmotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Config;
import java.util.function.DoubleSupplier;

public class DCMotorIOSim implements DCMotorIO {
  private final DCMotorSim sim;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
  private final SlewRateLimiter voltageLimiter = new SlewRateLimiter(2.5);

  public DCMotorIOSim(LinearSystem<N2, N1, N2> plant, DCMotor motor) {
    sim = new DCMotorSim(plant, motor);
  }

  public void updateInputs(DCMotorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(Config.LOOP_PERIOD_SEC);

    inputs.connected = true;

    inputs.rawPositionRad = sim.getAngularPositionRad();
    inputs.rawVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.rawAccelerationRadPerSecSq = sim.getAngularAccelerationRadPerSecSq();

    inputs.outputVoltageVolts = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
  }

  public void setPIDF(double kp, double ki, double kd, double ks, double kg) {
    pid.setPID(kp, ki, kd);
  }

  public void setPID(double kp, double ki, double kd) {
    pid.setPID(kp, ki, kd);
  }

  public void setMotionConstraints(double maxVelocityRadPerSec, double maxAccelerationRadPerSecSq) {
    pid.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelerationRadPerSecSq));
  }

  public void setPositionF(
      double positionRad,
      double velocityRadPerSec,
      double accelerationRadPerSecSq,
      double feedforward) {
    setVoltage(pid.calculate(sim.getAngularPositionRad(), positionRad));
  }

  public void setVelocityF(
      double velocityRadPerSec, double accelerationRadPerSecSq, double feedforward) {
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velocityRadPerSec));
  }

  public void setVoltage(double volts) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(voltageLimiter.calculate(volts));
    } else {
      voltageLimiter.reset(volts);
    }
  }

  public void setVoltage(DoubleSupplier volts) {
    setVoltage(volts.getAsDouble());
  }

  public void setCurrent(double amps) {
    // TODO
    setVoltage(amps);
  }

  public void resetPosition(double positionRad) {
    sim.setAngle(positionRad);
  }

  public void follow(DCMotorIO motor, Boolean isInverted) {
    // TODO
    setVoltage(() -> (isInverted ? -motor.getVoltage() : motor.getVoltage()));
  }

  public double getVoltage() {
    return sim.getInputVoltage();
  }
}
