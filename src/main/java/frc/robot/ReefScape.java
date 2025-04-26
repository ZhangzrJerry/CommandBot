package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utils.AllianceFlipUtil;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** 包含比赛场地相关的所有常量和配置 */
public class ReefScape {
  // 场地类型枚举
  @Getter
  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;
  }

  // AprilTag布局类型枚举
  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
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

  // 游戏道具相关常量
  public static class GamePiece {
    public enum Type {
      ALGAE,
      CORAL
    }

    public static class Algae {
      public static final double DIAMETER = 0.4064; // 16 inches in meters
      public static final double HEIGHT_HIGH = 1.115 + DIAMETER / 2.0;
      public static final double HEIGHT_LOW = 0.711 + DIAMETER / 2.0;
      public static final double LIMIT = 0.375;

      // 可得分位置
      public static final Pose3d[] SCORABLE_POSES =
          new Pose3d[] {
            Field.Processor.ALGAE_POSE,
            AllianceFlipUtil.flipPose(Field.Processor.ALGAE_POSE),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 0 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 1 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 2 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 3 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 4 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 6 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 7 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 8 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 0.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 1.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 2.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 3.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 4.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 5.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 6.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 7.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 0.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 1.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 2.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 3.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 4.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 5.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 6.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 7.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 0 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 1 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 2 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 3 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 4 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 6 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 7 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 8 * DIAMETER, true)
          };

      // 可拾取位置
      public static final Pose3d[] PICKABLE_POSES =
          new Pose3d[] {
            Field.Reef.getAlgaePose(18, HEIGHT_HIGH),
            Field.Reef.getAlgaePose(19, HEIGHT_LOW),
            Field.Reef.getAlgaePose(20, HEIGHT_HIGH),
            Field.Reef.getAlgaePose(21, HEIGHT_LOW),
            Field.Reef.getAlgaePose(22, HEIGHT_HIGH),
            Field.Reef.getAlgaePose(17, HEIGHT_LOW),
            Field.Reef.getAlgaePose(7, HEIGHT_HIGH),
            Field.Reef.getAlgaePose(8, HEIGHT_LOW),
            Field.Reef.getAlgaePose(9, HEIGHT_HIGH),
            Field.Reef.getAlgaePose(10, HEIGHT_LOW),
            Field.Reef.getAlgaePose(11, HEIGHT_HIGH),
            Field.Reef.getAlgaePose(6, HEIGHT_LOW),
            new Pose3d(Field.BLUE_X, Field.LOW_Y, DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
            new Pose3d(Field.BLUE_X, Field.MID_Y, DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
            new Pose3d(Field.BLUE_X, Field.HIGH_Y, DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
            new Pose3d(Field.RED_X, Field.LOW_Y, DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
            new Pose3d(Field.RED_X, Field.MID_Y, DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION),
            new Pose3d(Field.RED_X, Field.HIGH_Y, DIAMETER / 2.0 + Coral.LENGTH, Field.ROTATION)
          };
    }

    public static class Coral {
      public static final double LENGTH = 0.301625; // 11 + 7/8 inches in meters
      public static final double DIAMETER = 0.1016; // 4 inches in meters

      // 可得分位置
      public static final Pose3d[] SCORABLE_POSES =
          new Pose3d[] {
            Field.Reef.getCoralPose(18, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(18, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(18, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(18, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(18, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(18, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(19, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(19, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(19, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(19, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(19, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(19, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(20, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(20, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(20, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(20, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(20, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(20, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(21, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(21, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(21, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(21, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(21, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(21, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(22, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(22, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(22, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(22, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(22, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(22, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(17, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(17, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(17, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(17, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(17, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(17, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(7, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(7, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(7, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(7, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(7, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(7, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(8, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(8, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(8, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(8, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(8, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(8, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(9, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(9, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(9, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(9, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(9, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(9, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(10, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(10, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(10, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(10, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(10, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(10, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(11, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(11, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(11, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(11, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(11, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(11, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(6, true, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(6, true, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(6, true, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            Field.Reef.getCoralPose(6, false, Field.Reef.HEIGHT_LEVEL_4, Field.Reef.ANGLE_LEVEL_4),
            Field.Reef.getCoralPose(6, false, Field.Reef.HEIGHT_LEVEL_3, Field.Reef.ANGLE_LEVEL_3),
            Field.Reef.getCoralPose(6, false, Field.Reef.HEIGHT_LEVEL_2, Field.Reef.ANGLE_LEVEL_2),
            PoseUtils.getPoseBasedOnTag(
                6, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                6, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                6,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                6, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                6,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                7, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                7, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                7,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                7, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                7,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                8, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                8, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                8,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                8, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                8,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                9, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                9, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                9,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                9, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                9,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                10, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                10, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                10,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                10, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                10,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                11, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                11, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                11,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                11, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                11,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                17, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                17, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                17,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                17, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                17,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                18, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                18, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                18,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                18, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                18,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                19, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                19, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                19,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                19, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                19,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                20, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                20, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                20,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                20, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                20,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                21, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                21, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                21,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                21, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                21,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                22, -0.142, 0, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                22, -0.142, DIAMETER + 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                22,
                -0.142,
                (DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                22, -0.142, -DIAMETER - 0.034, Field.Reef.HEIGHT_LEVEL_1, Field.Reef.ANGLE_LEVEL_1),
            PoseUtils.getPoseBasedOnTag(
                22,
                -0.142,
                -(DIAMETER + 0.034) * 2,
                Field.Reef.HEIGHT_LEVEL_1,
                Field.Reef.ANGLE_LEVEL_1)
          };

      // 可拾取位置
      public static final Pose3d[] PICKABLE_POSES =
          new Pose3d[] {
            new Pose3d(Field.BLUE_X, Field.MID_Y, LENGTH, Field.ROTATION),
            new Pose3d(Field.RED_X, Field.MID_Y, LENGTH, Field.ROTATION),
            new Pose3d(Field.BLUE_X, Field.LOW_Y, LENGTH, Field.ROTATION),
            new Pose3d(Field.RED_X, Field.LOW_Y, LENGTH, Field.ROTATION),
            new Pose3d(Field.BLUE_X, Field.HIGH_Y, LENGTH, Field.ROTATION),
            new Pose3d(Field.RED_X, Field.HIGH_Y, LENGTH, Field.ROTATION),

            // 1
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 2
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 3
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 4
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 5
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 6
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 7
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 8
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 9
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 10
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 11
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 12
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 13
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 14
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 15
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 16
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 17
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 18
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 19
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),

            // 20
            PoseUtils.getPoseBasedOnTag(1, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(2, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(12, -0.08, 0, 1.1, Math.toRadians(35.0)),
            PoseUtils.getPoseBasedOnTag(13, -0.08, 0, 1.1, Math.toRadians(35.0)),
          };
    }

    /**
     * 根据选择的游戏部件类型和分支位置获取得分位置
     *
     * @param selectedGamePieceType 游戏部件类型
     * @param selectedBranch 分支位置
     * @return 得分位置
     */
    public static Pose2d getScorePoseBySelection(
        Type selectedGamePieceType, String selectedBranch) {
      switch (selectedGamePieceType) {
        case ALGAE:
          switch (selectedBranch) {
            case "A":
            case "B":
            case "AB":
              return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get("AB"));
            case "C":
            case "D":
            case "CD":
              return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get("CD"));
            case "E":
            case "F":
            case "EF":
              return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get("EF"));
            case "G":
            case "H":
            case "GH":
              return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get("GH"));
            case "I":
            case "J":
            case "IJ":
              return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get("IJ"));
            case "K":
            case "L":
            case "KL":
              return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get("KL"));
          }
          break;
        case CORAL:
          if (!selectedBranch.isEmpty()) {
            return AllianceFlipUtil.flipPose(Field.Reef.SCORE_POSES.get(selectedBranch));
          }
          break;
        default:
          break;
      }

      return new Pose2d();
    }
  }

  // 场地相关常量
  public static class Field {
    public static final double LENGTH = 17.548;
    public static final double WIDTH = 8.052;
    public static final double BLUE_X = 1.222391128540039;
    public static final double RED_X = 16.335620880126953;
    public static final double LOW_Y = 2.191911458969116;
    public static final double MID_Y = 4.0191121101379395;
    public static final double HIGH_Y = 5.851738929748535;
    public static final Rotation3d ROTATION = new Rotation3d(0.0, Math.PI / 2.0, 0.0);

    // 礁石区域相关常量
    public static class Reef {
      public static final Translation2d CENTER = new Translation2d(4.4893484, WIDTH / 2.0);
      public static final double FACE_LENGTH = 0.93453204;
      public static final double CENTER_TO_SIDE_DISTANCE = 1.178389072;

      // 高度相关常量
      public static final double HEIGHT_LEVEL_4 = 1.8288; // 72 inches
      public static final double HEIGHT_LEVEL_3 = 1.2096; // 47.625 inches
      public static final double HEIGHT_LEVEL_2 = 0.8096; // 31.875 inches
      public static final double HEIGHT_LEVEL_1 = 0.5080; // 20 inches

      // 角度相关常量
      public static final double ANGLE_LEVEL_4 = Math.PI / 2; // 90 degrees
      public static final double ANGLE_LEVEL_3 = Math.toRadians(35);
      public static final double ANGLE_LEVEL_2 = Math.toRadians(35);
      public static final double ANGLE_LEVEL_1 = Math.toRadians(25);

      // 位置调整常量
      public static final double SCORE_ADJUST_X = 0.429;
      public static final double SCORE_ADJUST_Y = 0.1643126;
      public static final double CORAL_ADJUST_X = -GamePiece.Coral.DIAMETER / 2.0;
      public static final double CORAL_ADJUST_Y = 0.1643126;
      public static final double ALGAE_ADJUST_X = -GamePiece.Algae.DIAMETER / 2.0 + 0.045;

      // 得分位置映射
      public static final Map<String, Pose2d> SCORE_POSES = new HashMap<>();

      static {
        SCORE_POSES.put("A", getScorePose(18, -SCORE_ADJUST_Y));
        SCORE_POSES.put("B", getScorePose(18, SCORE_ADJUST_Y));
        SCORE_POSES.put("AB", getScorePose(18, 0.0));

        SCORE_POSES.put("C", getScorePose(17, -SCORE_ADJUST_Y));
        SCORE_POSES.put("D", getScorePose(17, SCORE_ADJUST_Y));
        SCORE_POSES.put("CD", getScorePose(17, 0.0));

        SCORE_POSES.put("E", getScorePose(22, -SCORE_ADJUST_Y));
        SCORE_POSES.put("F", getScorePose(22, SCORE_ADJUST_Y));
        SCORE_POSES.put("EF", getScorePose(22, 0.0));

        SCORE_POSES.put("G", getScorePose(21, -SCORE_ADJUST_Y));
        SCORE_POSES.put("H", getScorePose(21, SCORE_ADJUST_Y));
        SCORE_POSES.put("GH", getScorePose(21, 0.0));

        SCORE_POSES.put("I", getScorePose(20, -SCORE_ADJUST_Y));
        SCORE_POSES.put("J", getScorePose(20, SCORE_ADJUST_Y));
        SCORE_POSES.put("IJ", getScorePose(20, 0.0));

        SCORE_POSES.put("K", getScorePose(19, -SCORE_ADJUST_Y));
        SCORE_POSES.put("L", getScorePose(19, SCORE_ADJUST_Y));
        SCORE_POSES.put("KL", getScorePose(19, 0.0));
      }

      private static Pose2d getScorePose(int id, double adjustY) {
        return getPoseBasedOnTag(id, SCORE_ADJUST_X, adjustY, new Rotation2d());
      }

      private static Pose2d getPoseBasedOnTag(
          int id, double adjustX, double adjustY, Rotation2d adjustHeading) {
        var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
        var shiftedPose = tagPose.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
        return new Pose2d(
            shiftedPose.getTranslation(), shiftedPose.getRotation().rotateBy(adjustHeading));
      }

      // 获取珊瑚位置
      public static Pose3d getCoralPose(int id, boolean isLeft, double height, double pitchRad) {
        return PoseUtils.getPoseBasedOnTag(
            id, CORAL_ADJUST_X, CORAL_ADJUST_Y * (isLeft ? -1.0 : 1.0), height, -pitchRad);
      }

      // 获取藻类位置
      public static Pose3d getAlgaePose(int id, double height) {
        return PoseUtils.getPoseBasedOnTag(id, ALGAE_ADJUST_X, 0.0, height, 0.0);
      }
    }

    // 驳船区域相关常量
    public static class Barge {
      public static final double NET_HEIGHT = 2.27;
      public static final double SCORE_X = 7.656325721740723;
      public static final Rotation2d SCORE_HEADING = Rotation2d.kZero;

      // 根据Y坐标获取位置
      public static Pose3d getPoseBasedOnY(double y, boolean closedToBlue) {
        return new Pose3d(
            Field.LENGTH / 2.0 + (closedToBlue ? 0.15 : -0.15),
            y,
            NET_HEIGHT - GamePiece.Algae.DIAMETER / 2.0,
            new Rotation3d());
      }
    }

    // 处理器区域相关常量
    public static class Processor {
      public static final double FACE_LENGTH = 0.7112; // 28 inches
      public static final Pose2d CENTER =
          new Pose2d(17.548 - 11.56081, 1.032, Rotation2d.fromDegrees(270));
      public static final Pose3d ALGAE_POSE =
          new Pose3d(
              CENTER.getX(),
              CENTER.getY() + 0.32,
              GamePiece.Algae.DIAMETER / 2.0 + 0.1,
              new Rotation3d());
      public static final Pose2d SCORE_POSE =
          new Pose2d(
              CENTER.getX(), CENTER.getY() - 0.48, CENTER.getRotation().plus(Rotation2d.kPi));
    }

    // 珊瑚站相关常量
    public static class CoralStation {
      public static final double FACE_LENGTH = 2.026138;
      public static final double HEIGHT = 0.7112; // 28 inches
      public static final double X = 0;
      public static final double COLLECT_ANGLE = 144.011;

      public static final Pose2d LEFT_CENTER_COLLECT_POSE =
          new Pose2d(
              1.0426816940307617,
              7.231509685516357,
              Rotation2d.fromDegrees(90 - COLLECT_ANGLE + 180.0));
      public static final Pose2d LEFT_LEFT_COLLECT_POSE =
          new Pose2d(
              1.5467664003372192,
              7.382765293121338,
              Rotation2d.fromDegrees(90 - COLLECT_ANGLE + 180.0));
      public static final Pose2d RIGHT_CENTER_COLLECT_POSE =
          new Pose2d(
              1.0387587547302246,
              0.8044585585594177,
              Rotation2d.fromDegrees(COLLECT_ANGLE - 90 + 180.0));
      public static final Pose2d RIGHT_RIGHT_COLLECT_POSE =
          new Pose2d(
              1.5467664003372192,
              (WIDTH - 7.382765293121338),
              Rotation2d.fromDegrees(COLLECT_ANGLE - 90 + 180.0));
    }
  }

  // 辅助方法
  public static class PoseUtils {
    public static Pose2d getPoseBasedOnTag(
        int id, double adjustX, double adjustY, Rotation2d adjustHeading) {
      var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
      var shiftedPose = tagPose.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
      return new Pose2d(
          shiftedPose.getTranslation(), shiftedPose.getRotation().rotateBy(adjustHeading));
    }

    public static Pose3d getPoseBasedOnTag(
        int id, double adjustX, double adjustY, double height, double pitchRad) {
      var tagPose2d = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
      var shiftedPose2d =
          tagPose2d.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
      return new Pose3d(
          shiftedPose2d.getX(),
          shiftedPose2d.getY(),
          height,
          new Rotation3d(0.0, pitchRad, tagPose2d.getRotation().getRadians()));
    }

    public static boolean isAlgaeHighPick(Pose2d pose) {
      if (pose.getX() < Field.LENGTH / 2.0) {
        Rotation2d angle = pose.getTranslation().minus(Field.Reef.CENTER).getAngle();
        if (angle.getSin() > 0.5 || angle.getSin() < -0.5) {
          return angle.getCos() > 0;
        } else {
          return angle.getCos() < 0;
        }
      } else {
        Rotation2d angle =
            pose.getTranslation()
                .minus(AllianceFlipUtil.flipTranslation(Field.Reef.CENTER))
                .getAngle();
        if (angle.getSin() > 0.5 || angle.getSin() < -0.5) {
          return angle.getCos() < 0;
        } else {
          return angle.getCos() > 0;
        }
      }
    }
  }

  public static final FieldType FIELD_TYPE = FieldType.ANDYMARK;
}
