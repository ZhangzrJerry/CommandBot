package frc.robot.interfaces.hardwares.sensors.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.interfaces.hardwares.CanDevice;
import frc.robot.utils.PhoenixConfigurator;
import frc.robot.utils.dashboard.Alert;
import lombok.Getter;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final Alert offlineAlert;

  @Getter private final StatusSignal<Angle> yawSignal;
  @Getter private final StatusSignal<AngularVelocity> omegaSignal;
  @Getter private final StatusSignal<Temperature> tempSignal;

  public GyroIOPigeon2(String name, CanDevice gyro) {
    pigeon = new Pigeon2(gyro.id(), gyro.bus());
    offlineAlert = new Alert(name + " offline!", Alert.AlertType.WARNING);

    yawSignal = pigeon.getYaw();
    omegaSignal = pigeon.getAngularVelocityZWorld();
    tempSignal = pigeon.getTemperature();

    PhoenixConfigurator.configure(
        "Gyro config", () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()));

    PhoenixConfigurator.configure("Gyro zero", () -> pigeon.getConfigurator().setYaw(0.0));

    PhoenixConfigurator.configure(
        "Gyro set yaw vel signal update frequency",
        () -> BaseStatusSignal.setUpdateFrequencyForAll(100., yawSignal, omegaSignal));

    PhoenixConfigurator.configure("Gyro optimize CAN utilization", pigeon::optimizeBusUtilization);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yawSignal, omegaSignal).isOK();
    offlineAlert.set(!inputs.connected);

    inputs.yaw = Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
    inputs.omegaRadPerSec = Units.degreesToRadians(omegaSignal.getValueAsDouble());

    inputs.temperature = tempSignal.getValueAsDouble();
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    pigeon.setYaw(yaw.getDegrees());
  }
}
