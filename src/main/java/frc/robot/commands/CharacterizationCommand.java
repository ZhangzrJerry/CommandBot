package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class CharacterizationCommand {
  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public static Command createKsCharacterizationCommand(
      String name,
      DoubleSupplier currentRampRateAmp,
      DoubleSupplier velocityThreshold,
      Consumer<Double> setCurrent,
      DoubleSupplier getVelocity) {
    final var state = new StaticCharacterizationState();
    final var timer = new Timer();

    return Commands.run(
            () -> {
              timer.start();
              state.characterizationOutput = currentRampRateAmp.getAsDouble() * timer.get();
              setCurrent.accept(state.characterizationOutput);
              Logger.recordOutput(
                  name + "/StaticCharacterizationCurrentOutputAmp", state.characterizationOutput);
            })
        .until(() -> getVelocity.getAsDouble() >= velocityThreshold.getAsDouble())
        .finallyDo(
            () -> {
              setCurrent.accept(0.0);
              timer.stop();
              Logger.recordOutput(
                  name + "/StaticCharacterizationCurrentOutputAmp", state.characterizationOutput);
              System.out.println("Calculated Ks: " + state.characterizationOutput + " amps");
            })
        .withName(name + " kS Characterization");
  }

  public static Command createWheelRadiusCharacterizationCommand(
      String name,
      DoubleSupplier characterizationSpeed,
      Consumer<Double> setAngularVelocity,
      DoubleSupplier getGyroYaw,
      DoubleSupplier getWheelPositions,
      DoubleSupplier wheelbaseDiagonal,
      Runnable endCharacterization) {
    final var state = new WheelRadiusCharacterizationState();
    final var omegaLimiter = new SlewRateLimiter(1.0);

    return Commands.run(
            () -> {
              setAngularVelocity.accept(
                  omegaLimiter.calculate(characterizationSpeed.getAsDouble()));

              double currentGyroYawRads = getGyroYaw.getAsDouble();
              state.sumGyroYawRads +=
                  MathUtil.angleModulus(currentGyroYawRads - state.lastGyroYawRads);
              state.lastGyroYawRads = currentGyroYawRads;

              var wheelPositions = getWheelPositions.getAsDouble();
              var avgWheelPosition = Math.abs(wheelPositions - state.startWheelPosition);

              state.currentEffectiveWheelRadius =
                  (state.sumGyroYawRads * (wheelbaseDiagonal.getAsDouble() / 2.0))
                      / avgWheelPosition;
              Logger.recordOutput(
                  name + "/WheelRadiusCharacterization/DrivePosition", avgWheelPosition);
              Logger.recordOutput(
                  name + "/WheelRadiusCharacterization/AccumGyroYawRads", state.sumGyroYawRads);
              Logger.recordOutput(
                  name + "/WheelRadiusCharacterization/CurrentWheelRadiusMeters",
                  state.currentEffectiveWheelRadius);
            })
        .finallyDo(
            () -> {
              endCharacterization.run();

              if (state.sumGyroYawRads <= Math.PI * 2.0) {
                System.out.println("No enough data for characterization");
              } else {
                System.out.println(
                    "Effective Wheel Radius: " + state.currentEffectiveWheelRadius + " meters");
              }
            })
        .withName(name + " Wheel Radius Characterization");
  }

  private static class WheelRadiusCharacterizationState {
    public double lastGyroYawRads = 0.0;
    public double sumGyroYawRads = 0.0;
    public double startWheelPosition = 0.0;
    public double currentEffectiveWheelRadius = 0.0;
  }
}
