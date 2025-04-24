package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.sensors.gyro.GyroIO;
import frc.robot.hardware.sensors.gyro.GyroIOInputsAutoLogged;
import frc.robot.hardware.sensors.odometry.OdometryThread;
import frc.robot.hardware.sensors.odometry.OdometryThread.WheeledObservation;
import frc.robot.utils.logging.AlertUtil;
import frc.robot.utils.logging.LoggedUtil;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import java.util.concurrent.ArrayBlockingQueue;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SwerveOdometry {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final AlertUtil gyroOfflineAlert = new AlertUtil("Gyro offline!", AlertUtil.AlertType.WARNING);

  ArrayBlockingQueue<WheeledObservation> odometryCachedWheeledObservationQueue;

  @Getter
  private UncertainPose2d pose = new UncertainPose2d(new Pose2d());
  private Rotation2d lastGyroYaw;
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  public SwerveOdometry(GyroIO gyroIO, ArrayBlockingQueue<WheeledObservation> observations) {
    this.gyroIO = gyroIO;
    this.odometryCachedWheeledObservationQueue = observations;
    this.odometryCachedWheeledObservationQueue.clear();

    // Initialize last module positions
    for (int i = 0; i < 4; i++) {
      lastModulePositions[i] = new SwerveModulePosition();
    }

    lastGyroYaw = gyroIO.getYaw();
  }

  public void updateInputs() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    gyroOfflineAlert.set(!gyroInputs.connected);

    var wheeledObservationArray = odometryCachedWheeledObservationQueue.toArray(WheeledObservation[]::new);
    odometryCachedWheeledObservationQueue.clear();

    for (int i = 0; i < wheeledObservationArray.length; i++) {
      updatePose(wheeledObservationArray[i]);
    }

    Logger.recordOutput("Swerve/Odometry/EstimatedPose", pose.getPose());
    LoggedUtil.logMatrix("Swerve/Odometry/EstimatedPoseCovariance", pose.getCovariance());
  }

  private void updatePose(OdometryThread.WheeledObservation observation) {
    SwerveModulePosition[] modulePositions = observation.wheelPositions();
    Rotation2d gyroYaw = observation.yaw();

    Logger.recordOutput("Swerve/Odometry/ModulePositions", modulePositions);
    Twist2d twist = SwerveConfig.SWERVE_KINEMATICS.toTwist2d(lastModulePositions, modulePositions);

    double translationError = Math.hypot(twist.dx, twist.dy) * 0.01;
    double rotationError = Math.abs(twist.dtheta) * 0.01;

    if (gyroYaw != null) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
      rotationError *= 0.5;
      lastGyroYaw = gyroYaw;
    }

    pose.setPose(pose.getPose().exp(twist));
    pose.setXVariance(pose.getXVariance() + translationError);
    pose.setYVariance(pose.getYVariance() + translationError);
    pose.setThetaVariance(pose.getThetaVariance() + rotationError);
  }

  public void resetGyroHeading(Rotation2d heading) {
    pose.setPose(new Pose2d(pose.getPose().getTranslation(), heading));
    pose.setThetaVariance(0);
    pose.setXThetaCovariance(pose.getXThetaCovariance() * 0.1);
    pose.setYThetaCovariance(pose.getYThetaCovariance() * 0.1);
    if (gyroIO.getYaw() != null) {
      lastGyroYaw = heading;
      gyroIO.setYaw(heading);
    }
  }

  public void resetWheeledPose(UncertainPose2d pose) {
    this.pose.setPose(pose.getPose());
    this.pose.setCovariance(pose.getCovariance());
    if (gyroIO.getYaw() != null) {
      lastGyroYaw = pose.getPose().getRotation();
      gyroIO.setYaw(lastGyroYaw);
    }
  }
}
