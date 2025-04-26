package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

public class EndeffectorConfig {
  static TalonFXConfiguration getAlgaeTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  static TalonFXConfiguration getCoralTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  static CANdiConfiguration getCandiConfig() {
    var config = new CANdiConfiguration();

    config.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
    config.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;

    return config;
  }
}
