package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.hardwares.motors.DCMotorIO;
import frc.robot.interfaces.hardwares.motors.DCMotorIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.motors.DCMotorIOSim;
import frc.robot.interfaces.hardwares.motors.DCMotorIOTalonfx;
import frc.robot.services.VisualizeService;
import frc.robot.utils.Gains.GainsImpl;
import frc.robot.utils.dashboard.AlertManager;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.UnitConverter;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  @RequiredArgsConstructor
  public enum ClimberGoal {
    IDLE(
        new TunableNumber("Climber/IdleVoltageVolt", 0.0),
        new TunableNumber("Climber/IdleMinAngleDegree", -180.0)),
    SAFE_HOME(
        new TunableNumber("Climber/SafeHomeVoltageVolt", 3.0),
        new TunableNumber("Climber/SafeHomeMinAngleDegree", -45.0)),
    READY(
        new TunableNumber("Climber/ReadyVoltageVolt", 9.0),
        new TunableNumber("Climber/ReadyMinAngleDegree", 90.0));

    private final DoubleSupplier voltageVoltSupplier;
    private final DoubleSupplier positionDegreeSupplier;

    double getVoltageVolt() {
      return voltageVoltSupplier.getAsDouble();
    }

    double getPositionRad() {
      return Units.degreesToRadians(positionDegreeSupplier.getAsDouble());
    }
  }

  private final DCMotorIO io;
  private final DCMotorIOInputsAutoLogged inputs = new DCMotorIOInputsAutoLogged();
  private final AlertManager offlineAlert =
      new AlertManager("Climber motor offline!", AlertManager.AlertType.WARNING);

  @Getter
  @Setter
  @AutoLogOutput(key = "Climber/Goal")
  private ClimberGoal goal = ClimberGoal.SAFE_HOME;

  @Getter
  @Setter
  @AutoLogOutput(key = "Climber/IsClimbing")
  private Boolean isClimbing = false;

  private boolean wantPull = false;

  @Override
  public void periodic() {
    updateInputs();

    offlineAlert.set(!inputs.connected);

    if (inputs.appliedPosition >= goal.getPositionRad()) {
      if (goal == ClimberGoal.READY
          && wantPull
          && inputs.appliedPosition < ClimberConfig.pullPositionLimitDegree.get()) {
        io.setVoltage(ClimberConfig.pullVoltageVolt.get());
      } else {
        stop();
      }
    } else {
      io.setVoltage(goal.getVoltageVolt());
    }
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setTeleopInput(boolean wantPull) {
    this.wantPull = wantPull;
  }

  public void stop() {
    io.stop();
  }

  public void registerVisualize(VisualizeService visualizer) {
    visualizer.registerVisualizeComponent(
        Constants.Ascope.Component.CLIMBER,
        Constants.Ascope.Component.DRIVETRAIN,
        () ->
            ClimberConfig.ZEROED_CLIMBER_TF.plus(
                new Transform3d(0, 0, 0, new Rotation3d(0, 0, inputs.appliedPosition - 165.0))));
  }

  private Climber(DCMotorIO io) {
    this.io = io;
  }

  public static Climber createReal() {
    return new Climber(
        new DCMotorIOTalonfx(
            "Climber",
            Constants.Ports.Can.CLIMBER,
            ClimberConfig.getTalonConfig(),
            UnitConverter.scale(360).withUnits("rot", "deg")));
  }

  public static Climber createSim() {
    return new Climber(
        new DCMotorIOSim(
            DCMotor.getKrakenX60Foc(1),
            0.35,
            ClimberConfig.REDUCTION,
            UnitConverter.scale(180.0 / Math.PI).withUnits("rad", "deg"),
            new GainsImpl(1.0, 0.0, 0.0, 0.0, 0.0)));
  }

  public static Climber createIO() {
    return new Climber(new DCMotorIO() {});
  }
}
