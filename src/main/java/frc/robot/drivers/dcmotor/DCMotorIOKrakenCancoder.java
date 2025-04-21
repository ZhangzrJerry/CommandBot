package frc.robot.drivers.dcmotor;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.utils.CanDevice;
import frc.robot.utils.PhoenixHelper;

public class DCMotorIOKrakenCancoder extends DCMotorIOKraken {
  CANcoder cancoder;

  public DCMotorIOKrakenCancoder(
      String name,
      CanDevice device,
      TalonFXConfiguration motorConfig,
      CanDevice coder,
      CANcoderConfiguration coderConfig) {
    super(name, device, motorConfig);
    this.cancoder = new CANcoder(coder.id(), coder.bus());
    withCancoder(name, coder.id());
  }
}
