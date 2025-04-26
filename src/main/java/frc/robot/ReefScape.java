package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.ReefScape.GamePiece.Algae;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class ReefScape {
  public static class GamePiece {
    public static enum Type {
      ALGAE,
      CORAL
    }

    public static class Algae {
      public static final double DIAMETER = 0.4064; // 16 inches in meters

      public static final Pose3d[] scorablePose = new Pose3d[] {
          Field.Processor.PROCESSOR_ALGAE_POSE,
          AllianceFlipUtil.flipPose(Field.Processor.PROCESSOR_ALGAE_POSE),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 0 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 1 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 2 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 3 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 4 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 5 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 6 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 7 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + 0.35 + 8 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 0 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 1 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 2 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 3 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 4 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 5 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 6 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 7 * DIAMETER),
          Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - 0.35 - 8 * DIAMETER)
      };

      public static final Pose3d[] pickablePose = new Pose3d[] {
          Field.Reef.getAlgaePose(18, Field.Reef.AlGAE_HIGH_HEIGHT),
          Field.Reef.getAlgaePose(19, Field.Reef.AlGAE_LOW_HEIGHT),
          Field.Reef.getAlgaePose(20, Field.Reef.AlGAE_HIGH_HEIGHT),
          Field.Reef.getAlgaePose(21, Field.Reef.AlGAE_LOW_HEIGHT),
          Field.Reef.getAlgaePose(22, Field.Reef.AlGAE_HIGH_HEIGHT),
          Field.Reef.getAlgaePose(17, Field.Reef.AlGAE_LOW_HEIGHT),
          Field.Reef.getAlgaePose(7, Field.Reef.AlGAE_HIGH_HEIGHT),
          Field.Reef.getAlgaePose(8, Field.Reef.AlGAE_LOW_HEIGHT),
          Field.Reef.getAlgaePose(9, Field.Reef.AlGAE_HIGH_HEIGHT),
          Field.Reef.getAlgaePose(10, Field.Reef.AlGAE_LOW_HEIGHT),
          Field.Reef.getAlgaePose(11, Field.Reef.AlGAE_HIGH_HEIGHT),
          Field.Reef.getAlgaePose(6, Field.Reef.AlGAE_LOW_HEIGHT),
          new Pose3d(Field.BLUE_X, Field.LOW_Y, Algae.DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
          new Pose3d(Field.BLUE_X, Field.MID_Y, Algae.DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
          new Pose3d(Field.BLUE_X, Field.HIGH_Y, Algae.DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
          new Pose3d(Field.RED_X, Field.LOW_Y, Algae.DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
          new Pose3d(Field.RED_X, Field.MID_Y, Algae.DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
          new Pose3d(Field.RED_X, Field.HIGH_Y, Algae.DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
      };
    }

    public static class Coral {
      public static final double LENGTH = 0.301625; // 11 + 7/8 inches in meters
      public static final double DIAMETER = 0.1016; // 4 inches in meters

      private static final double ADJUST_X = -0.142;
      private static final double INTERVAl = 0.034;

      public static final Pose3d[] scorablePose = new Pose3d[] {
          Field.Reef.getCoralPose(18, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(18, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(18, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(18, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(18, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(18, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(19, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(19, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(19, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(19, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(19, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(19, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(20, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(20, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(20, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(20, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(20, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(20, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(21, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(21, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(21, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(21, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(21, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(21, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(22, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(22, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(22, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(22, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(22, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(22, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(17, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(17, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(17, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(17, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(17, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(17, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(7, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(7, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(7, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(7, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(7, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(7, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(8, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(8, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(8, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(8, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(8, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(8, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(9, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(9, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(9, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(9, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(9, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(9, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(10, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(10, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(10, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(10, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(10, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(10, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(11, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(11, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(11, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(11, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(11, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(11, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getCoralPose(6, true, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(6, true, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(6, true, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),
          Field.Reef.getCoralPose(6, false, Field.Reef.L4_HEIGHT_METERS, Field.Reef.L4_ANGLE_RADIANS),
          Field.Reef.getCoralPose(6, false, Field.Reef.L3_HEIGHT_METERS, Field.Reef.L3_ANGLE_RADIANS),
          Field.Reef.getCoralPose(6, false, Field.Reef.L2_HEIGHT_METERS, Field.Reef.L2_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(6, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(6, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(6, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(6, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(6, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(7, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(7, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(7, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(7, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(7, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(8, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(8, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(8, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(8, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(8, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(9, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(9, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(9, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(9, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(9, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(10, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(10, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(10, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(10, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(10, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(11, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(11, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(11, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(11, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(11, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(17, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(17, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(17, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(17, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(17, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(18, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(18, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(18, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(18, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(18, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(19, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(19, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(19, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(19, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(19, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(20, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(20, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(20, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(20, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(20, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(21, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(21, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(21, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(21, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(21, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),

          Field.Reef.getPoseBasedOnTag(22, ADJUST_X, 0, Field.Reef.L1_HEIGHT_METERS, Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(22, ADJUST_X, DIAMETER + INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(22, ADJUST_X, (DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(22, ADJUST_X, -DIAMETER - INTERVAl, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS),
          Field.Reef.getPoseBasedOnTag(22, ADJUST_X, -(DIAMETER + INTERVAl) * 2, Field.Reef.L1_HEIGHT_METERS,
              Field.Reef.L1_ANGLE_RADIANS) };

      private static final double CORAL_ADJUST_X = -0.2;
      private static final double CORAL_ADJUST_Z = 0.45;
      private static final double CORAL_ADJUST_Y = 0.02;
      private static final double CORAL_ADJUST_INTERVAL = DIAMETER + 0.015;
      private static final double CORAL_ADJUST_PITCH = 0.4363323129985824;
      public static final Pose3d[] pickablePose = new Pose3d[] {
          new Pose3d(Field.BLUE_X, Field.LOW_Y, LENGTH / 2.0, Field.ROTATION),
          new Pose3d(Field.BLUE_X, Field.MID_Y, LENGTH / 2.0, Field.ROTATION),
          new Pose3d(Field.BLUE_X, Field.HIGH_Y, LENGTH / 2.0, Field.ROTATION),
          new Pose3d(Field.RED_X, Field.LOW_Y, LENGTH / 2.0, Field.ROTATION),
          new Pose3d(Field.RED_X, Field.MID_Y, LENGTH / 2.0, Field.ROTATION),
          new Pose3d(Field.RED_X, Field.HIGH_Y, LENGTH / 2.0, Field.ROTATION),

      };
    }
  }

  public static class Field {
    public static final double LENGTH = 17.548;
    public static final double WIDTH = 8.052;
    public static final double BLUE_X = 1.222391128540039;
    public static final double RED_X = 16.335620880126953;
    public static final double LOW_Y = 2.191911458969116;
    public static final double MID_Y = 4.0191121101379395;
    public static final double HIGH_Y = 5.851738929748535;
    public static final Rotation3d ROTATION = new Rotation3d(0.0, Math.PI / 2.0, 0.0);

    public static class Reef {
      public static final Translation2d CENTER = new Translation2d(4.4893484, WIDTH / 2.0); // 176.746 inches in meters
      public static final double FACE_LENGTH = 0.93453204; // 36.792600 inches in meters
      public static final double CENTER_2_SIDE_DISTANCE = 1.178389072;

      public static final Map<String, Pose2d> SCORE_POSES = new HashMap<>();
      public static final double SCORE_ADJUST_X = 0.429;
      public static final double CORAL_ADJUST_Y_OFFSET = -0.01;
      public static final double SCORE_ADJUST_Y = 0.1643126; // 6.469 inches in meters
      public static final Rotation2d SCORE_ADJUST_HEADING = Rotation2d.kZero;
      public static final Map<String, Pose3d> CORAL_POSES = new HashMap<>();
      public static final double CORAL_ADJUST_X = -GamePiece.Coral.DIAMETER / 2.0;
      public static final double CORAL_ADJUST_Y = 0.1643126; // 6.469 inches in meters
      public static final double AlGAE_ADJUST_X = -GamePiece.Algae.DIAMETER / 2.0 + 0.045;

      public static final double L4_HEIGHT_METERS = 1.8288; // 72 inches in meters
      public static final double L3_HEIGHT_METERS = 1.2096; // 47.625 inches in meters
      public static final double L2_HEIGHT_METERS = 0.8096; // 31.875 inches in meters
      public static final double L1_HEIGHT_METERS = 0.5080; // 20 inches in meters
      public static final double L4_ANGLE_RADIANS = 1.5707963267948966; // 90 degrees in radians
      public static final double L3_ANGLE_RADIANS = 0.6108652381980153; // 35 degrees in radians
      public static final double L2_ANGLE_RADIANS = 0.6108652381980153; // 35 degrees in radians
      public static final double L1_ANGLE_RADIANS = 0.4363323129985824; // -25 degrees in radians
      public static final double AlGAE_HIGH_HEIGHT = 1.115 + GamePiece.Algae.DIAMETER / 2.0;
      public static final double AlGAE_LOW_HEIGHT = 0.711 + GamePiece.Algae.DIAMETER / 2.0;

      private static Pose2d getScorePose(int id, double adjustY) {
        return getPoseBasedOnTag(id, SCORE_ADJUST_X, adjustY, SCORE_ADJUST_HEADING);
      }

      private static Pose3d getCoralPose(int id, boolean isLeft, double height, double pitchRad) {
        return getPoseBasedOnTag(
            id,
            CORAL_ADJUST_X,
            CORAL_ADJUST_Y * (isLeft ? -1.0 : 1.0),
            height,
            -pitchRad);
      }

      private static Pose3d getAlgaePose(int id, double height) {
        return getPoseBasedOnTag(id, AlGAE_ADJUST_X, 0.0, height, 0.0);
      }

      private static Pose2d getPoseBasedOnTag(
          int id, double adjustX, double adjustY, Rotation2d adjustHeading) {
        var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
        var shiftedPose = tagPose.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
        return new Pose2d(
            shiftedPose.getTranslation(), shiftedPose.getRotation().rotateBy(adjustHeading));
      }

      private static Pose3d getPoseBasedOnTag(
          int id, double adjustX, double adjustY, double height, double pitchRad) {
        var tagPose2d = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
        var shiftedPose2d = tagPose2d.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
        return new Pose3d(
            shiftedPose2d.getX(),
            shiftedPose2d.getY(),
            height,
            new Rotation3d(0.0, pitchRad, tagPose2d.getRotation().getRadians()));
      }
    }

    public static class Barge {
      public static final double NET_HEIGHT = 2.27;
      public static final double BARGE_SCORE_X = 7.656325721740723;
      public static final Rotation2d BARGE_SCORE_HEADING = Rotation2d.kZero;

      public static Pose3d getPoseBasedOnY(double y) {
        return new Pose3d(
            Field.LENGTH / 2.0,
            y,
            NET_HEIGHT - Algae.DIAMETER / 2.0,
            new Rotation3d());
      }
    }

    public static class Processor {
      public static final double FACE_LENGTH = 0.7112; // 28 inches in meters

      public static final Pose2d PROCESSOR_CENTER = new Pose2d(11.56081, 8.05561, Rotation2d.fromDegrees(90));
      public static final Pose3d PROCESSOR_ALGAE_POSE = new Pose3d(
          PROCESSOR_CENTER.getX(),
          PROCESSOR_CENTER.getY() + 0.32,
          Algae.DIAMETER / 2.0 + 0.1,
          new Rotation3d());
      public static final Pose2d PROCESSOR_SCORE_POSE = new Pose2d(
          PROCESSOR_CENTER.getX(),
          PROCESSOR_CENTER.getY() - 0.48,
          PROCESSOR_CENTER.getRotation().plus(Rotation2d.kPi));
    }

    public static class CoralStation {
      public static final double FACE_LENGTH = 2.026138;
      public static final double HEIGHT = 0.7112; // 28 inches in meters
      public static final double x = 0;

      public static final Pose2d LEFT_CENTER_COLLECT_POSE = new Pose2d(
          1.0426816940307617, 7.231509685516357, Rotation2d.fromDegrees(90 - 144.011 + 180.0));
      public static final Pose2d LEFT_LEFT_COLLECT_POSE = new Pose2d(
          1.5467664003372192, 7.382765293121338, Rotation2d.fromDegrees(90 - 144.011 + 180.0));

      public static final Pose2d RIGHT_CENTER_COLLECT_POSE = new Pose2d(
          1.0387587547302246, 0.8044585585594177, Rotation2d.fromDegrees(144.011 - 90 + 180.0));
      public static final Pose2d RIGHT_RIGHT_COLLECT_POSE = new Pose2d(
          1.5467664003372192,
          (8.052 - 7.382765293121338),
          Rotation2d.fromDegrees(144.011 - 90 + 180.0));

      public static final Pose3d getCoralPose(double d) {
        return new Pose3d(
            LEFT_CENTER_COLLECT_POSE.getX() + d * Math.cos(144.011 * Math.PI / 180),
            LEFT_CENTER_COLLECT_POSE.getY() - d * Math.sin(144.011 * Math.PI / 180),
            CoralStation.FACE_LENGTH / 2.0,
            new Rotation3d(0, 0, 2.5));
      }
    }
  }

  public static final FieldType FIELD_TYPE = FieldType.ANDYMARK;

  @Getter
  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;
  }

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef");

    AprilTagLayoutType(String name) {
      try {
        layout = new AprilTagFieldLayout(
            Path.of(
                Filesystem.getDeployDirectory().getPath(),
                "apriltags",
                FIELD_TYPE.getJsonFolder(),
                name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException("Failed to serialize AprilTag layout JSON " + this);
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
