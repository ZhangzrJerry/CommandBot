package frc.robot.utils.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.Setter;

public class PoseUtil {
  public static class UncertainPose2d {
    @Getter @Setter private Pose2d pose;
    @Getter @Setter private Matrix<N3, N3> covariance;

    /**
     * 创建一个具有确定位姿的不确定位姿（协方差为0）
     *
     * @param pose 位姿
     */
    public UncertainPose2d(Pose2d pose) {
      this(pose, Matrix.eye(Nat.N3()));
    }

    /**
     * 创建一个具有不确定性的位姿
     *
     * @param pose 位姿
     * @param covariance 协方差矩阵
     */
    public UncertainPose2d(Pose2d pose, Matrix<N3, N3> covariance) {
      this.pose = pose;
      this.covariance = covariance;
    }

    public UncertainPose2d(Pose2d pose, double xVariance, double yVariance, double thetaVariance) {
      this.pose = pose;
      this.covariance = Matrix.eye(Nat.N3());
      this.covariance.set(0, 0, xVariance);
      this.covariance.set(1, 1, yVariance);
      this.covariance.set(2, 2, thetaVariance);
    }

    public double getXVariance() {
      return covariance.get(0, 0);
    }

    public double getYVariance() {
      return covariance.get(1, 1);
    }

    public double getThetaVariance() {
      return covariance.get(2, 2);
    }

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

    public void setXVariance(double xVariance) {
      covariance.set(0, 0, xVariance);
    }

    public void setYVariance(double yVariance) {
      covariance.set(1, 1, yVariance);
    }

    public void setThetaVariance(double thetaVariance) {
      covariance.set(2, 2, thetaVariance);
    }

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

    public void setYXCovariance(double yxCovariance) {
      covariance.set(1, 0, yxCovariance);
      covariance.set(0, 1, yxCovariance);
    }

    public void setThetaXCovariance(double thetaXCovariance) {
      covariance.set(2, 0, thetaXCovariance);
      covariance.set(0, 2, thetaXCovariance);
    }

    public void setThetaYCovariance(double thetaYCovariance) {
      covariance.set(2, 1, thetaYCovariance);
      covariance.set(1, 2, thetaYCovariance);
    }

    public double getPositionUncertainty() {
      return Math.sqrt(getXVariance() + getYVariance());
    }

    public double getAngleUncertainty() {
      return Math.sqrt(getThetaVariance());
    }

    public boolean isBetterThan(UncertainPose2d other) {
      double thisTrace = covariance.get(0, 0) + covariance.get(1, 1) + covariance.get(2, 2);
      double otherTrace =
          other.covariance.get(0, 0) + other.covariance.get(1, 1) + other.covariance.get(2, 2);
      return thisTrace < otherTrace;
    }

    public boolean isEquivalentTo(UncertainPose2d other, double sigma) {
      return mahalanobisDistance(other) < sigma;
    }

    /**
     * 计算与另一个位姿之间的马氏距离
     *
     * @param other 另一个位姿
     * @return 马氏距离
     */
    public double mahalanobisDistance(UncertainPose2d other) {
      Matrix<N3, N3> diff = new Matrix<>(N3.instance, N3.instance);
      diff.set(0, 0, pose.getX() - other.pose.getX());
      diff.set(1, 0, pose.getY() - other.pose.getY());
      diff.set(2, 0, pose.getRotation().minus(other.pose.getRotation()).getRadians());

      Matrix<N3, N3> totalCovariance = covariance.plus(other.covariance);
      Matrix<N3, N3> invCovariance = totalCovariance.inv();

      return Math.sqrt(diff.transpose().times(invCovariance).times(diff).get(0, 0));
    }

    @Override
    public String toString() {
      return String.format(
          "UncertainPose2d(X: %3f ± %3f, Y: %3f ± %3f, θ: %3f ± %3f)",
          pose.getX(),
          Math.sqrt(getXVariance()),
          pose.getY(),
          Math.sqrt(getYVariance()),
          pose.getRotation().getDegrees(),
          Units.radiansToDegrees(Math.sqrt(getThetaVariance())));
    }

    public void fuseWith(UncertainPose2d other) {
      // Matrix<N3, N3> totalCovariance = covariance.plus(other.covariance);
      // Matrix<N3, N3> invCovariance = totalCovariance.inv();

      // Matrix<N3, N3> fusedPose =
      // invCovariance.times(pose).plus(invCovariance.times(other.pose));
      // pose = new Pose2d(fusedPose.get(0, 0), fusedPose.get(1, 0), fusedPose.get(2,
      // 0));

      // covariance = totalCovariance;
    }
  }

  private PoseUtil() {}
}
