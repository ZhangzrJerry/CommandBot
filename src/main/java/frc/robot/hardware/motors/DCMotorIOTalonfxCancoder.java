package frc.robot.hardware.motors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.hardware.communication.CanDevice;
import frc.robot.utils.math.UnitConverter;

public class DCMotorIOTalonfxCancoder extends DCMotorIOTalonfx {
  CANcoder cancoder;

  public DCMotorIOTalonfxCancoder(
      String name,
      CanDevice device,
      TalonFXConfiguration motorConfig,
      CanDevice coder,
      CANcoderConfiguration coderConfig,
      UnitConverter ratioConverter,
      UnitConverter... offsetConverter) {
    super(name, device, motorConfig, ratioConverter, offsetConverter);
    this.cancoder = new CANcoder(coder.id(), coder.bus());
    withCancoder(name, coder.id());
  }
}
