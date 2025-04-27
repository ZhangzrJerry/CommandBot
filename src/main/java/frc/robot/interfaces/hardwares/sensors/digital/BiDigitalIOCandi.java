package frc.robot.interfaces.hardwares.sensors.digital;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import frc.robot.interfaces.hardwares.CanDevice;
import frc.robot.utils.PhoenixConfigurator;
import frc.robot.utils.dashboard.Alert;
import frc.robot.utils.dashboard.Alert.AlertType;

public class BiDigitalIOCandi implements BiDigitalIO {
  private final CANdi candi;

  private final StatusSignal<S1StateValue> s1;
  private final StatusSignal<S2StateValue> s2;
  private final Alert offlineAlert;

  public BiDigitalIOCandi(String name, CanDevice device, CANdiConfiguration config) {
    candi = new CANdi(device.id(), device.bus());
    PhoenixConfigurator.configure(
        "[" + name + "] config", () -> candi.getConfigurator().apply(config), 5);

    s1 = candi.getS1State();
    s2 = candi.getS2State();

    offlineAlert = new Alert(name + " offline!", AlertType.WARNING);
  }

  @Override
  public void updateInputs(BiDigitalIOInputs inputs) {
    inputs.connected = StatusSignal.refreshAll(s1, s2).isOK();
    offlineAlert.set(!inputs.connected);

    inputs.value1 = s1.getValue().equals(S1StateValue.High);
    inputs.value2 = s2.getValue().equals(S2StateValue.High);
  }

  @Override
  public boolean getValue1() {
    return s1.getValue().equals(S1StateValue.High);
  }

  @Override
  public boolean getValue2() {
    return s2.getValue().equals(S2StateValue.High);
  }
}
