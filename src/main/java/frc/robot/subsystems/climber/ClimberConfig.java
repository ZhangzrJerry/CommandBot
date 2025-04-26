package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.dashboard.TunableNumber;

public class ClimberConfig {
  protected static final TunableNumber pullPositionLimitDegree =
      new TunableNumber("Climber/PullPositionLimitDegree", 180.0);

  protected static final TunableNumber pullVoltageVolt =
      new TunableNumber("Climber/PullVoltageVolt", 3.0);

  static double HOME_POSITION_DEGREE = -80.0;
  static double REDUCTION = (72.0 / 24.0) * (34.0 / 12.0) * 49.0;

  static TalonFXConfiguration getTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 250.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = REDUCTION;

    return config;
  }

  public static final Transform3d ZEROED_CLIMBER_TF =
      new Transform3d(
          0,
          0.3005,
          0.354,
          new Rotation3d(0, Math.PI / 2.0, 0).plus(new Rotation3d(-0.24589798, 0, 0)));
}
