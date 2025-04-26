package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.dashboard.TunableNumber;

class IntakeConfig {
  protected static final TunableNumber slamVelocityThreshDegreePerSec =
      new TunableNumber("Intake/Slam/VelocityThreshDegreePerSec", 15.0);
  protected static final TunableNumber slamTimeSecs =
      new TunableNumber("Intake/Slam/TimeSecs", 0.05);
  protected static final TunableNumber slamUpCurrentAmp =
      new TunableNumber("Intake/Slam/UpCurrentAmp", -20.0);
  protected static final TunableNumber slamDownCurrentAmp =
      new TunableNumber("Intake/Slam/DownCurrentAmp", 20.0);

  static final double ARM_REDUCTION = (32.0 / 12.0) * (72.0 / 12.0);
  static final double ARM_HOME_POSITION_DEGREE = 102.0;

  static TalonFXConfiguration getRollerTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  static TalonFXConfiguration getArmTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = ARM_REDUCTION;

    return config;
  }

  static final Transform3d ZEROED_INTAKE_TF =
      new Transform3d(0.299, 0, 0.1695, new Rotation3d(1.0, 0, Math.PI / 2));
}
