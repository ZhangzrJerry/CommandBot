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
      CanDevice talonfx,
      TalonFXConfiguration motorConfig,
      CanDevice cancoder,
      CANcoderConfiguration coderConfig,
      UnitConverter ratioConverter,
      UnitConverter... offsetConverter) {
    super(name, talonfx, motorConfig, ratioConverter, offsetConverter);
    this.cancoder = new CANcoder(cancoder.id(), cancoder.bus());
    withCancoder(name, cancoder.id());
  }
}
