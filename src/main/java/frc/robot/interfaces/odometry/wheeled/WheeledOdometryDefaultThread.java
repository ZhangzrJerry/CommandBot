package frc.robot.interfaces.odometry.wheeled;

import java.util.concurrent.ArrayBlockingQueue;

public interface WheeledOdometryThread {
  default ArrayBlockingQueue<WheeledObservation> start() {
    return new ArrayBlockingQueue<WheeledObservation>(0);
  }
}
