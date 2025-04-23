package frc.robot.hardware.communication.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues,
 * with support for cases where gyro might be null.
 */
public class PhoenixAsyncSensorThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  // private static boolean isCANFD = new
  // CANBus(Ports.Can.CHASSIS_CANIVORE_BUS).isNetworkFD();
  private static boolean isCANFD = true;
  private static PhoenixAsyncSensorThread instance = null;

  public static PhoenixAsyncSensorThread getInstance() {
    if (instance == null) {
      instance = new PhoenixAsyncSensorThread();
    }
    return instance;
  }

  private PhoenixAsyncSensorThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /**
   * Registers a Phoenix signal to be read from the thread.
   *
   * @param signal The signal to register (can be null)
   * @return Queue for the signal's values
   */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    try {
      if (signal != null) {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
        System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
        newSignals[phoenixSignals.length] = signal;
        phoenixSignals = newSignals;
        phoenixQueues.add(queue);
      } else {
        // For null signals, add a queue that will remain empty
        phoenixQueues.add(queue);
      }
    } finally {
      signalsLock.unlock();
    }
    return queue;
  }

  /**
   * Registers a generic signal to be read from the thread.
   *
   * @param signal Supplier for the signal value (can be null)
   * @return Queue for the signal's values
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    try {
      if (signal != null) {
        genericSignals.add(signal);
        genericQueues.add(queue);
      } else {
        // For null suppliers, add a queue that will remain empty
        genericQueues.add(queue);
      }
    } finally {
      signalsLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      signalsLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (phoenixSignals.length > 0) {
          if (isCANFD) {
            BaseStatusSignal.waitForAll(2.0 / 250.0, phoenixSignals);
          } else {
            Thread.sleep((long) (1000.0 / 250.0));
            BaseStatusSignal.refreshAll(phoenixSignals);
          }
        } else {
          // If no phoenix signals, just sleep at the desired rate
          Thread.sleep((long) (1000.0 / 250.0));
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      signalsLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;

        // Only calculate latency if we have phoenix signals
        if (phoenixSignals.length > 0) {
          double totalLatency = 0.0;
          for (BaseStatusSignal signal : phoenixSignals) {
            if (signal != null) {
              totalLatency += signal.getTimestamp().getLatency();
            }
          }
          timestamp -= totalLatency / phoenixSignals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < phoenixQueues.size(); i++) {
          if (i < phoenixSignals.length && phoenixSignals[i] != null) {
            phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
          }
          // Else: queue remains empty (for null signals)
        }

        for (int i = 0; i < genericQueues.size(); i++) {
          if (i < genericSignals.size() && genericSignals.get(i) != null) {
            genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
          }
          // Else: queue remains empty (for null suppliers)
        }

        for (Queue<Double> timestampQueue : timestampQueues) {
          timestampQueue.offer(timestamp);
        }
      } finally {
        signalsLock.unlock();
      }
    }
  }
}
