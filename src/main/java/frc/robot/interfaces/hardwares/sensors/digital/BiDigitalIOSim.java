package frc.robot.interfaces.hardwares.sensors.digital;

import java.util.function.BooleanSupplier;

public class BiDigitalIOSim implements BiDigitalIO {
  private final BooleanSupplier value1Supplier;
  private final BooleanSupplier value2Supplier;

  public BiDigitalIOSim(BooleanSupplier value1Supplier, BooleanSupplier value2Supplier) {
    this.value1Supplier = value1Supplier;
    this.value2Supplier = value2Supplier;
  }

  @Override
  public void updateInputs(BiDigitalIOInputs inputs) {
    inputs.connected = true;
    inputs.value1 = value1Supplier.getAsBoolean();
    inputs.value2 = value2Supplier.getAsBoolean();
  }

  @Override
  public boolean getValue1() {
    return value1Supplier.getAsBoolean();
  }

  @Override
  public boolean getValue2() {
    return value2Supplier.getAsBoolean();
  }
}
