// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.hardwares.sensors.camera.AtagVisionIO;
import frc.robot.interfaces.hardwares.sensors.camera.AtagVisionIOInputsAutoLogged;
import frc.robot.interfaces.hardwares.sensors.camera.AtagVisionIOPhoton;
import frc.robot.interfaces.hardwares.sensors.camera.AtagVisionIOPhotonSim;
import frc.robot.utils.dashboard.AlertManager;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AtagVision extends SubsystemBase {
  private final AtagVisionIO[] io;
  private final AtagVisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  @Getter private double latestTimestamp = 0.0;

  @Getter
  private UncertainPose2d latestPose =
      new UncertainPose2d(new Pose2d(), 0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f);

  private final AlertManager visionOfflineAlert =
      new AlertManager("Vision offline!", AlertManager.AlertType.WARNING);

  public static AtagVision createReal() {
    return new AtagVision(
        new AtagVisionIOPhoton(
            VisionConfig.REEF_BACK_LEFT_NAME,
            VisionConfig.REEF_BACK_LEFT_IN_ROBOT,
            VisionConfig.aprilTagLayout),
        new AtagVisionIOPhoton(
            VisionConfig.REEF_BACK_RIGHT_NAME,
            VisionConfig.REEF_BACK_RIGHT_IN_ROBOT,
            VisionConfig.aprilTagLayout));
  }

  public static AtagVision createIO() {
    return new AtagVision(new AtagVisionIO() {}, new AtagVisionIO() {});
  }

  public static AtagVision createSim(Supplier<Pose2d> pose) {
    return new AtagVision(
        new AtagVisionIOPhotonSim(
            VisionConfig.REEF_BACK_LEFT_NAME,
            VisionConfig.REEF_BACK_LEFT_IN_ROBOT,
            pose,
            VisionConfig.aprilTagLayout),
        new AtagVisionIOPhotonSim(
            VisionConfig.REEF_BACK_RIGHT_NAME,
            VisionConfig.REEF_BACK_RIGHT_IN_ROBOT,
            pose,
            VisionConfig.aprilTagLayout));
  }

  private AtagVision(AtagVisionIO... io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new AtagVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AtagVisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < disconnectedAlerts.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    visionOfflineAlert.set(!inputs[0].connected || !inputs[1].connected);

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = VisionConfig.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > VisionConfig.maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConfig.maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > VisionConfig.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > VisionConfig.aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConfig.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConfig.angularStdDevBaseline * stdDevFactor;
        if (observation.type() == AtagVisionIO.PoseObservationType.MEGATAG_2) {
          linearStdDev *= VisionConfig.linearStdDevMegatag2Factor;
          angularStdDev *= VisionConfig.angularStdDevMegatag2Factor;
        }
        if (cameraIndex < VisionConfig.cameraStdDevFactors.length) {
          linearStdDev *= VisionConfig.cameraStdDevFactors[cameraIndex];
          angularStdDev *= VisionConfig.cameraStdDevFactors[cameraIndex];
        }

        // Update estimated pose and covariance
        if (robotPosesAccepted.size() > 0) {
          Pose3d bestPose = robotPosesAccepted.get(robotPosesAccepted.size() - 1);
          latestPose.setPose(
              new Pose2d(bestPose.getX(), bestPose.getY(), bestPose.getRotation().toRotation2d()));
          latestPose.setXVariance(linearStdDev * linearStdDev);
          latestPose.setYVariance(linearStdDev * linearStdDev);
          latestPose.setThetaVariance(angularStdDev * angularStdDev);
          latestTimestamp = observation.timestamp();
        }
      }

      // Log camera data
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }
}
