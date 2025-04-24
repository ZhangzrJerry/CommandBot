package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.services.ServiceManager;
import frc.robot.services.Visualization;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private final RobotContainer robotContainer;
  private final ServiceManager serviceManager = ServiceManager.getInstance();
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  private Command autoCmd;
  private Command initCmd;

  private boolean autoMessagePrinted;
  private double autoStart;

  public Robot() {
    configureCtreLogger();
    configureAkitLogger();

    registerService();

    RobotController.setBrownoutVoltage(6.0);
    robotContainer = new RobotContainer();

    initCmd = robotContainer.getInitializationCommand();

    if (initCmd != null) {
      initCmd.schedule();
    }
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

    commandScheduler.run();
    serviceManager.updateAll();

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
    commandScheduler.cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void configureCtreLogger() {
    if (Robot.isReal()) {
      SignalLogger.enableAutoLogging(false);
      SignalLogger.stop();
    }
  }

  private void configureAkitLogger() {
    if (Robot.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      if (Constants.IS_LIVE_DEBUG) {
        Logger.addDataReceiver(new NT4Publisher());
      }
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

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

    commandScheduler.onCommandInitialize(
        (Command command) -> {
          System.out.println("" + command.getName() + "");
          logCommandFunction.accept(command, true);
        });

    commandScheduler.onCommandFinish(
        (Command command) -> {
          System.out.println("\u001B[32m" + command.getName() + "\u001B[0m");
          logCommandFunction.accept(command, false);
        });

    commandScheduler.onCommandInterrupt(
        (Command command) -> {
          System.out.println("\u001B[31m" + command.getName() + "\u001B[0m");
          logCommandFunction.accept(command, false);
        });
  }

  private void registerService() {
    serviceManager.registerService(new Visualization());
    serviceManager.initAll();
  }
}
