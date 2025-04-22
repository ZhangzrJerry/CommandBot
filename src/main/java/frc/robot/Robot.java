package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.virtuals.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private final RobotContainer robotContainer;
  private Command autoCmd;

  private boolean autoMessagePrinted;
  private double autoStart;

  public Robot() {
    configureCtreLogger();
    configureAkitLogger();

    RobotController.setBrownoutVoltage(6.0);
    robotContainer = new RobotContainer();
  }

  private void configureCtreLogger() {
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
  }

  private void configureAkitLogger() {
    if (Robot.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      if (Config.IS_LIVE_DEBUG) {
        Logger.addDataReceiver(new NT4Publisher());
      }
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }
    // case REPLAY -> {
    // setUseTiming(false);
    // var logPath = LogFileUtil.findReplayLog();
    // Logger.setReplaySource(new WPILOGReader(logPath));
    // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
    // "_sim")));
    // }

    Logger.start();

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };

    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              System.out.println(command.getName() + " started");
              logCommandFunction.accept(command, true);
            });

    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              System.out.println(command.getName() + " finished");
              logCommandFunction.accept(command, false);
            });

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              System.out.println(command.getName() + " interrupted");
              logCommandFunction.accept(command, false);
            });
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    if (autoCmd != null) {
      if (!autoCmd.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    CommandScheduler.getInstance().run();
    VirtualSubsystem.periodicAll();

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCmd = robotContainer.getAutonomousCommand();

    if (autoCmd != null) {
      autoCmd.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCmd != null) {
      autoCmd.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
