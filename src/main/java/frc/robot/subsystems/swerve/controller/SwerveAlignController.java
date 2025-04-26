package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ReefScape.Field;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.dashboard.TunableGains.TunablePidGains;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.GeomUtil;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveAlignController implements SwerveController {
        private static final TunableNumber maxLineupShiftingYMeter = new TunableNumber(
                        "Swerve/AlignController/MaxLineupShiftingYMeter", 1.5);
        private static final TunableNumber maxLineupShiftingXMeter = new TunableNumber(
                        "Swerve/AlignController/MaxLineupShiftingXMeter", 1.0);

        private static final TunablePidGains translationGains = new TunablePidGains(
                        "Swerve/AlignController/TranslationGains", 2.1, 0.0, 0.1);
        private static final TunableNumber translationToleranceMeter = new TunableNumber(
                        "Swerve/AlignController/TranslationToleranceMeter", 0.08);
        private static final double maxTranslationVelMeterPerSec = SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC;

        private static final TunablePidGains rotationGains = new TunablePidGains("Swerve/AlignController/RotationGains",
                        4.0, 0.0, 0.0);
        private static final TunableNumber rotationToleranceDegree = new TunableNumber(
                        "Swerve/AlignController/RotationToleranceDegree", 5.0);
        private static final double maxRotationVelRadPerSec = SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC;

        @AutoLogOutput(key = "Swerve/AlignController/GoalPose")
        private final Supplier<Pose2d> goalPoseSupplier;
        private final Supplier<Pose2d> currentPoseSupplier;
        private final PIDController translationController = new PIDController(0.0, 0.0, 0.0);
        private final PIDController rotationController = new PIDController(0.0, 0.0, 0.0);

        private boolean hasDone = false;
        private boolean hasHeadingAtGoal = false;
        private boolean hasTranslationAtGoal = false;

        public SwerveAlignController(
                        Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> currentPoseSupplier) {
                this.goalPoseSupplier = goalPoseSupplier;
                this.currentPoseSupplier = currentPoseSupplier;
                rotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public ChassisSpeeds getChassisSpeeds() {
                translationController.setP(translationGains.getKP());
                translationController.setD(translationGains.getKD());
                translationController.setTolerance(translationToleranceMeter.get());

                rotationController.setP(rotationGains.getKP());
                rotationController.setD(rotationGains.getKD());
                rotationController.setTolerance(rotationToleranceDegree.get());

                var shiftedGoalPose = getShiftedGoalPose();
                var currentPose = currentPoseSupplier.get();
                Logger.recordOutput("Swerve/AlignController/GoalPose", shiftedGoalPose);

                var currentDistance = currentPose.getTranslation().getDistance(shiftedGoalPose.getTranslation());
                Logger.recordOutput("Swerve/AlignController/TranslationErrorMeter", currentDistance);

                var translationDir = currentPose.getTranslation().minus(shiftedGoalPose.getTranslation()).getAngle();
                var translationFeedback = translationController.calculate(currentDistance, 0.0);

                var rotationErrorDegree = currentPose.getRotation().minus(shiftedGoalPose.getRotation()).getDegrees();
                Logger.recordOutput("Swerve/AlignController/RotationErrorDegree", rotationErrorDegree);

                hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= rotationToleranceDegree.get();
                hasTranslationAtGoal = Math.abs(currentDistance) <= translationToleranceMeter.get();
                hasDone = hasHeadingAtGoal && hasTranslationAtGoal;

                if (hasDone) {
                        return new ChassisSpeeds();
                }

                var translationOutputScalar = hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;
                Logger.recordOutput("Swerve/AlignController/TranslationOutputScalar", translationOutputScalar);

                var translationVel = new Translation2d(
                                MathUtil.clamp(
                                                translationFeedback * translationOutputScalar,
                                                -maxTranslationVelMeterPerSec,
                                                maxTranslationVelMeterPerSec),
                                translationDir);

                var rotationVel = MathUtil.clamp(
                                rotationController.calculate(
                                                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                                                MathUtil.angleModulus(shiftedGoalPose.getRotation().getRadians())),
                                -maxRotationVelRadPerSec,
                                maxRotationVelRadPerSec);

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                                translationVel.getX(), translationVel.getY(), rotationVel, currentPose.getRotation());
        }

        private Pose2d getShiftedGoalPose() {
                var goalPose = goalPoseSupplier.get();
                var currentPose = currentPoseSupplier.get();

                var offset = currentPose.relativeTo(goalPose);
                var yDistance = Math.abs(offset.getY());
                var xDistance = Math.abs(offset.getX());

                var shiftXT = MathUtil.clamp(
                                (yDistance / (Field.CoralStation.FACE_LENGTH * 2.0))
                                                + ((xDistance - 0.3) / (Field.CoralStation.FACE_LENGTH * 3.0)),
                                0.0,
                                1.0);

                var shiftYT = MathUtil.clamp(offset.getX() / Field.CoralStation.FACE_LENGTH, 0.0, 1.0);

                var flippedShiftedGoalPose = goalPose.transformBy(
                                GeomUtil.toTransform2d(
                                                -shiftXT * maxLineupShiftingXMeter.get(),
                                                Math.copySign(shiftYT * maxLineupShiftingYMeter.get() * 0.8,
                                                                offset.getY())));

                return new Pose2d(flippedShiftedGoalPose.getTranslation(), goalPose.getRotation());
        }

        @Override
        public Boolean headingAtGoal() {
                return hasHeadingAtGoal;
        }

        @Override
        public Boolean translationAtGoal() {
                return hasTranslationAtGoal;
        }

        @Override
        public Boolean translationErrorWithin(double tolerance) {
                return Math.abs(
                                currentPoseSupplier
                                                .get()
                                                .getTranslation()
                                                .getDistance(goalPoseSupplier.get().getTranslation())) <= tolerance;
        }

        @Override
        public String getName() {
                return "Align Controller";
        }
}
