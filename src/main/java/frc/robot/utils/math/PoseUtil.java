package frc.robot.utils.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.Setter;

public class PoseUtil {
  /**
   * Represents a 2D pose with associated uncertainty (covariance matrix). This is useful for
   * tracking pose estimates with varying levels of confidence.
   */
  public static class UncertainPose2d {
    @Getter @Setter private Pose2d pose;
    @Getter @Setter private Matrix<N3, N3> covariance;

    /**
     * Creates an uncertain pose with zero covariance (perfectly certain pose).
     *
     * @param pose The pose
     */
    public UncertainPose2d(Pose2d pose) {
      this(
          pose,
          Matrix.eye(Nat.N3())
              .times(1e-9)); // Small epsilon instead of zero for numerical stability
    }

    /**
     * Creates an uncertain pose with full covariance matrix.
     *
     * @param pose The pose
     * @param covariance The 3x3 covariance matrix (x, y, theta)
     */
    public UncertainPose2d(Pose2d pose, Matrix<N3, N3> covariance) {
      this.pose = pose;
      this.covariance = covariance;
    }

    /**
     * Creates an uncertain pose with diagonal covariance (no cross-correlation).
     *
     * @param pose The pose
     * @param xVariance Variance in x direction (meters^2)
     * @param yVariance Variance in y direction (meters^2)
     * @param thetaVariance Variance in angle (radians^2)
     */
    public UncertainPose2d(Pose2d pose, double xVariance, double yVariance, double thetaVariance) {
      this.pose = pose;
      this.covariance = Matrix.eye(Nat.N3());
      this.covariance.set(0, 0, xVariance);
      this.covariance.set(1, 1, yVariance);
      this.covariance.set(2, 2, thetaVariance);
    }

    // Variance getters
    public double getXVariance() {
      return covariance.get(0, 0);
    }

    public double getYVariance() {
      return covariance.get(1, 1);
    }

    public double getThetaVariance() {
      return covariance.get(2, 2);
    }

    // Covariance getters
    public double getXYCovariance() {
      return covariance.get(0, 1);
    }

    public double getXThetaCovariance() {
      return covariance.get(0, 2);
    }

    public double getYThetaCovariance() {
      return covariance.get(1, 2);
    }

    public double getYXCovariance() {
      return covariance.get(1, 0);
    }

    public double getThetaXCovariance() {
      return covariance.get(2, 0);
    }

    public double getThetaYCovariance() {
      return covariance.get(2, 1);
    }

    // Variance setters
    public void setXVariance(double xVariance) {
      covariance.set(0, 0, xVariance);
    }

    public void setYVariance(double yVariance) {
      covariance.set(1, 1, yVariance);
    }

    public void setThetaVariance(double thetaVariance) {
      covariance.set(2, 2, thetaVariance);
    }

    // Covariance setters (symmetric matrix)
    public void setXYCovariance(double xyCovariance) {
      covariance.set(0, 1, xyCovariance);
      covariance.set(1, 0, xyCovariance);
    }

    public void setXThetaCovariance(double xThetaCovariance) {
      covariance.set(0, 2, xThetaCovariance);
      covariance.set(2, 0, xThetaCovariance);
    }

    public void setYThetaCovariance(double yThetaCovariance) {
      covariance.set(1, 2, yThetaCovariance);
      covariance.set(2, 1, yThetaCovariance);
    }

    /**
     * Gets the total position uncertainty (combined x and y) in meters.
     *
     * @return Combined position standard deviation
     */
    public double getPositionUncertainty() {
      return Math.sqrt(getXVariance() + getYVariance());
    }

    /**
     * Gets the angle uncertainty in radians.
     *
     * @return Angle standard deviation
     */
    public double getAngleUncertainty() {
      return Math.sqrt(getThetaVariance());
    }

    /**
     * Checks if this pose estimate is better (more certain) than another.
     *
     * @param other The other pose to compare with
     * @return True if this pose has lower total variance
     */
    public boolean isBetterThan(UncertainPose2d other) {
      return getTotalVariance() < other.getTotalVariance();
    }

    /**
     * Gets the total variance (trace of covariance matrix).
     *
     * @return Sum of diagonal elements
     */
    public double getTotalVariance() {
      return covariance.get(0, 0) + covariance.get(1, 1) + covariance.get(2, 2);
    }

    /**
     * Checks if two poses are statistically equivalent within given sigma bounds.
     *
     * @param other The other pose to compare with
     * @param sigma The number of standard deviations to consider
     * @return True if poses are within sigma bounds of each other
     */
    public boolean isEquivalentTo(UncertainPose2d other, double sigma) {
      return mahalanobisDistance(other) < sigma;
    }

    /**
     * Computes the Mahalanobis distance to another pose. This accounts for the uncertainty in both
     * poses.
     *
     * @param other The other pose
     * @return Mahalanobis distance between the poses
     */
    public double mahalanobisDistance(UncertainPose2d other) {
      Matrix<N3, N1> diff = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
      diff.set(0, 0, pose.getX() - other.pose.getX());
      diff.set(1, 0, pose.getY() - other.pose.getY());
      diff.set(2, 0, pose.getRotation().minus(other.pose.getRotation()).getRadians());

      Matrix<N3, N3> totalCovariance = covariance.plus(other.covariance);
      Matrix<N3, N3> invCovariance = totalCovariance.inv();

      return Math.sqrt(diff.transpose().times(invCovariance).times(diff).get(0, 0));
    }

    /**
     * Transforms this pose by the given uncertain transform and propagates the uncertainty. This
     * properly accounts for both the pose's uncertainty and the transform's uncertainty.
     *
     * @param transform The uncertain transform to apply
     * @return The new transformed uncertain pose
     */
    public UncertainPose2d transformBy(UncertainTransform2d transform) {
      // Transform the nominal pose
      Pose2d newPose = pose.transformBy(transform.getTransform());

      // Jacobian with respect to the original pose (J_pose)
      Matrix<N3, N3> jacobianPose = Matrix.eye(Nat.N3());
      double cosTheta = pose.getRotation().getCos();
      double sinTheta = pose.getRotation().getSin();
      double tx = transform.getTransform().getX();
      double ty = transform.getTransform().getY();

      // Partial derivatives of transformed x,y with respect to original theta
      jacobianPose.set(0, 2, -tx * sinTheta - ty * cosTheta);
      jacobianPose.set(1, 2, tx * cosTheta - ty * sinTheta);

      // Jacobian with respect to the transform (J_transform)
      Matrix<N3, N3> jacobianTransform = Matrix.eye(Nat.N3());
      jacobianTransform.set(0, 2, -ty); // ∂x'/∂θ_transform
      jacobianTransform.set(1, 2, tx); // ∂y'/∂θ_transform

      // Propagate both sources of uncertainty:
      // Cov_new = J_pose * Cov_pose * J_pose^T + J_transform * Cov_transform *
      // J_transform^T
      Matrix<N3, N3> newCovariance =
          jacobianPose
              .times(covariance)
              .times(jacobianPose.transpose())
              .plus(
                  jacobianTransform
                      .times(transform.getCovariance())
                      .times(jacobianTransform.transpose()));

      return new UncertainPose2d(newPose, newCovariance);
    }

    /**
     * Computes the relative transform between this pose and another, propagating uncertainty.
     *
     * @param other The other pose
     * @return The relative transform with uncertainty
     */
    public UncertainPose2d relativeTo(UncertainPose2d other) {
      Pose2d dif = pose.relativeTo(other.getPose());

      // Jacobian of the relative transform operation
      Matrix<N3, N3> jacobian = Matrix.eye(Nat.N3());
      double cosTheta = other.pose.getRotation().getCos();
      double sinTheta = other.pose.getRotation().getSin();
      double dx = pose.getX() - other.pose.getX();
      double dy = pose.getY() - other.pose.getY();

      jacobian.set(0, 0, cosTheta);
      jacobian.set(0, 1, sinTheta);
      jacobian.set(1, 0, -sinTheta);
      jacobian.set(1, 1, cosTheta);
      jacobian.set(0, 2, -dx * sinTheta + dy * cosTheta);
      jacobian.set(1, 2, -dx * cosTheta - dy * sinTheta);

      // Combined covariance
      Matrix<N3, N3> totalCovariance = covariance.plus(other.covariance);
      Matrix<N3, N3> newCovariance = jacobian.times(totalCovariance).times(jacobian.transpose());

      return new UncertainPose2d(dif, newCovariance);
    }

    /**
     * Fuses this pose estimate with another using Kalman filter update.
     *
     * @param other The other pose estimate to fuse with
     * @return The fused pose estimate
     */
    public UncertainPose2d fuseWith(UncertainPose2d other) {
      Matrix<N3, N3> invCov1 = covariance.inv();
      Matrix<N3, N3> invCov2 = other.covariance.inv();
      Matrix<N3, N3> fusedCov = (invCov1.plus(invCov2)).inv();

      Matrix<N3, N1> poseVec1 = new Matrix<>(Nat.N3(), Nat.N1());
      poseVec1.set(0, 0, pose.getX());
      poseVec1.set(1, 0, pose.getY());
      poseVec1.set(2, 0, pose.getRotation().getRadians());

      Matrix<N3, N1> poseVec2 = new Matrix<>(Nat.N3(), Nat.N1());
      poseVec2.set(0, 0, other.pose.getX());
      poseVec2.set(1, 0, other.pose.getY());
      poseVec2.set(2, 0, other.pose.getRotation().getRadians());

      Matrix<N3, N1> fusedPoseVec =
          fusedCov.times(invCov1.times(poseVec1).plus(invCov2.times(poseVec2)));

      Pose2d fusedPose =
          new Pose2d(
              fusedPoseVec.get(0, 0),
              fusedPoseVec.get(1, 0),
              new Rotation2d(fusedPoseVec.get(2, 0)));

      return new UncertainPose2d(fusedPose, fusedCov);
    }

    /**
     * Computes the Euclidean distance to another pose (ignoring uncertainty).
     *
     * @param other The other pose
     * @return Distance in meters
     */
    public double getDistanceTo(Pose2d other) {
      return pose.getTranslation().getDistance(other.getTranslation());
    }

    /**
     * Computes the angular difference to another pose (ignoring uncertainty).
     *
     * @param other The other pose
     * @return Angular difference in radians
     */
    public double getAngleTo(Pose2d other) {
      return pose.getRotation().minus(other.getRotation()).getRadians();
    }

    @Override
    public String toString() {
      return String.format(
          "UncertainPose2d(X: %.3f ± %.3f m, Y: %.3f ± %.3f m, θ: %.1f ± %.1f°)",
          pose.getX(),
          Math.sqrt(getXVariance()),
          pose.getY(),
          Math.sqrt(getYVariance()),
          pose.getRotation().getDegrees(),
          Units.radiansToDegrees(Math.sqrt(getThetaVariance())));
    }
  }

  /** Represents a 2D transform with associated uncertainty. */
  public static class UncertainTransform2d {
    @Getter private final Transform2d transform;
    @Getter private final Matrix<N3, N3> covariance;

    public UncertainTransform2d(Transform2d transform, Matrix<N3, N3> covariance) {
      this.transform = transform;
      this.covariance = covariance;
    }

    public UncertainTransform2d(
        Transform2d transform, double xVariance, double yVariance, double thetaVariance) {
      this.transform = transform;
      this.covariance = Matrix.eye(Nat.N3());
      this.covariance.set(0, 0, xVariance);
      this.covariance.set(1, 1, yVariance);
      this.covariance.set(2, 2, thetaVariance);
    }
  }

  private PoseUtil() {}
}
