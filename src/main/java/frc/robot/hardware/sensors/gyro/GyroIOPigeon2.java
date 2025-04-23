package frc.robot.hardware.sensors.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.hardware.communication.CanDevice;
import frc.robot.hardware.communication.phoenix.PhoenixConfigurator;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> omega;
  private final StatusSignal<Temperature> temp;

  public GyroIOPigeon2(CanDevice gyro) {
    pigeon = new Pigeon2(gyro.id(), gyro.bus());

    yaw = pigeon.getYaw();
    omega = pigeon.getAngularVelocityZWorld();
    temp = pigeon.getTemperature();

    PhoenixConfigurator.configure(
        "Gyro config", () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()));

    PhoenixConfigurator.configure("Gyro zero", () -> pigeon.getConfigurator().setYaw(0.0));

    PhoenixConfigurator.configure(
        "Gyro set yaw vel signal update frequency",
        () -> BaseStatusSignal.setUpdateFrequencyForAll(100., yaw, omega));

    PhoenixConfigurator.configure("Gyro optimize CAN utilization", pigeon::optimizeBusUtilization);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, omega).isOK();

    inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.omegaRadPerSec = Units.degreesToRadians(omega.getValueAsDouble());

    inputs.temperature = temp.getValueAsDouble();
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
  }
}
