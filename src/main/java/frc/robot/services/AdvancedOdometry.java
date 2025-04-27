// package frc.robot.services;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.interfaces.services.PoseService;
// import frc.robot.utils.dashboard.TunableNumber;
// import frc.robot.utils.math.PoseUtil.UncertainPose2d;
// import java.util.Map;
// import java.util.TreeMap;
// import lombok.Getter;
// import org.littletonrobotics.junction.Logger;

// public class BetterOdometry implements PoseService {
// @Getter
// private ServiceState state = ServiceState.STOPPED;
// @Getter private String errorMessage = "";

// // Using TreeMap for automatic sorting by timestamp
// private final TreeMap<Double, UncertainPose2d> poseHistory = new TreeMap<>();
// private final TunableNumber historyLength = new
// TunableNumber("Odometry/HistoryLength", 1.5);
// private final TunableNumber velocityCalculationWindow = new
// TunableNumber("Odometry/VelocityWindow", 0.1);

// private Pose2d currentPose = new Pose2d();
// private Translation2d currentVelocity = new Translation2d();
// private Rotation2d currentHeading = new Rotation2d();
// private double lastUpdateTime = 0;

// // Standard deviations for pose estimation
// private final Matrix<N3, N1> defaultStdDevs = Matrix.mat(Nat.N3(),
// Nat.N1()).fill(0.05, 0.05, 0.01); // x, y, theta in meters/radians

// public BetterOdometry() {}

// @Override
// public String getName() {
// return "Odometry";
// }

// @Override
// public void setState(ServiceState state) {
// this.state = state;
// }

// @Override
// public void setError(String errorMessage) {
// this.errorMessage = errorMessage;
// setState(ServiceState.ERROR);
// Logger.recordOutput("Services/Odometry/Error", true);
// }

// @Override
// public void init() {
// resetPose(new Pose2d());
// setState(ServiceState.RUNNING);
// Logger.recordOutput("Services/Odometry/Error", false);
// }

// @Override
// public void update() {
// try {
// double currentTime = Timer.getFPGATimestamp();

// // Only update if enough time has passed (20ms default loop time)
// if (currentTime - lastUpdateTime < 0.02 && !poseHistory.isEmpty()) {
// return;
// }

// lastUpdateTime = currentTime;

// // Update current pose
// updateCurrentPose();

// // Calculate velocity
// updateVelocity(currentTime);

// // Log data
// logData();

// // Clean up old data
// cleanupOldData();
// } catch (Exception e) {
// setError("Update error: " + e.getMessage());
// Logger.recordException("OdometryUpdateError", e);
// }
// }

// private void logData() {
// // Record to SmartDashboard
// SmartDashboard.putNumber("Odometry/X", currentPose.getX());
// SmartDashboard.putNumber("Odometry/Y", currentPose.getY());
// SmartDashboard.putNumber("Odometry/Heading",
// currentPose.getRotation().getDegrees());
// SmartDashboard.putNumber("Odometry/VelocityX", currentVelocity.getX());
// SmartDashboard.putNumber("Odometry/VelocityY", currentVelocity.getY());
// SmartDashboard.putNumber("Odometry/VelocityMag", currentVelocity.getNorm());

// // Record to Logger
// Logger.recordOutput("Services/Odometry/Pose", currentPose);
// Logger.recordOutput("Services/Odometry/Velocity", currentVelocity);
// Logger.recordOutput("Services/Odometry/Heading", currentHeading);
// Logger.recordOutput("Services/Odometry/PoseHistorySize", poseHistory.size());
// }

// @Override
// public Pose2d getCurrentPose() {
// return currentPose;
// }

// @Override
// public Rotation2d getCurrentHeading() {
// return currentHeading;
// }

// @Override
// public Translation2d getCurrentVelocity() {
// return currentVelocity;
// }

// @Override
// public double getPositionAccuracy() {
// if (poseHistory.isEmpty()) return 0.1; // Default 10cm accuracy if no data
// return poseHistory.lastEntry().getValue().getUncertainty().get(0, 0); //
// Return x std dev
// }

// @Override
// public double getHeadingAccuracy() {
// if (poseHistory.isEmpty()) return Math.toRadians(5.0); // Default 5 degree
// accuracy if no data
// return poseHistory.lastEntry().getValue().getUncertainty().get(2, 0); //
// Return theta std dev
// }

// @Override
// public void resetPose(Pose2d pose) {
// resetPose(pose, defaultStdDevs);
// }

// public void resetPose(Pose2d pose, Matrix<N3, N1> stdDevs) {
// currentPose = pose;
// currentHeading = pose.getRotation();
// poseHistory.clear();
// poseHistory.put(Timer.getFPGATimestamp(), new UncertainPose2d(pose,
// stdDevs));
// Logger.recordOutput("Services/Odometry/ResetPose", pose);
// }

// @Override
// public void resetHeading(Rotation2d heading) {
// resetHeading(heading, defaultStdDevs.get(2, 0));
// }

// public void resetHeading(Rotation2d heading, double headingStdDev) {
// currentHeading = heading;
// currentPose = new Pose2d(currentPose.getTranslation(), heading);

// // Update the latest pose in history with new heading
// if (!poseHistory.isEmpty()) {
// Map.Entry<Double, UncertainPose2d> lastEntry = poseHistory.lastEntry();
// Pose2d newPose = new Pose2d(lastEntry.getValue().getPose().getTranslation(),
// heading);

// // Update uncertainty matrix with new heading std dev
// Matrix<N3, N1> newUncertainty = lastEntry.getValue().getUncertainty().copy();
// newUncertainty.set(2, 0, headingStdDev);

// poseHistory.put(lastEntry.getKey(), new UncertainPose2d(newPose,
// newUncertainty));
// }

// Logger.recordOutput("Services/Odometry/ResetHeading", heading);
// }

// @Override
// public void addTransformObservation(TransformObservation observation) {
// try {
// double startTime = observation.startTimestamp();
// double endTime = observation.endTimestamp();
// Transform2d transform = observation.transform();
// Matrix<N3, N1> stdDevs = observation.stdDevs();

// // If history is empty, initialize with current pose
// if (poseHistory.isEmpty()) {
// poseHistory.put(startTime, new UncertainPose2d(currentPose, defaultStdDevs));
// }

// // Get the pose at start time (or closest available)
// Map.Entry<Double, UncertainPose2d> startEntry =
// poseHistory.floorEntry(startTime);
// if (startEntry == null) {
// startEntry = poseHistory.firstEntry();
// }

// // Calculate end pose
// Pose2d endPose = startEntry.getValue().getPose().plus(transform);

// // Propagate uncertainty (simplified - could use more sophisticated
// covariance propagation)
// Matrix<N3, N1> endUncertainty =
// startEntry.getValue().getUncertainty().plus(stdDevs);

// // Add to history
// poseHistory.put(endTime, new UncertainPose2d(endPose, endUncertainty));

// Logger.recordOutput("Services/Odometry/TransformObservation", transform);
// } catch (Exception e) {
// setError("Transform observation error: " + e.getMessage());
// Logger.recordException("TransformObservationError", e);
// }
// }

// @Override
// public void addPoseObservation(PoseObservation observation) {
// try {
// double timestamp = observation.timestamp();
// Pose2d pose = observation.pose();
// Matrix<N3, N1> stdDevs = observation.stdDevs();

// // Update history
// poseHistory.put(timestamp, new UncertainPose2d(pose, stdDevs));

// Logger.recordOutput("Services/Odometry/PoseObservation", pose);
// } catch (Exception e) {
// setError("Pose observation error: " + e.getMessage());
// Logger.recordException("PoseObservationError", e);
// }
// }

// private void updateCurrentPose() {
// if (poseHistory.isEmpty()) return;

// // Get the most recent pose
// Map.Entry<Double, UncertainPose2d> latestEntry = poseHistory.lastEntry();
// currentPose = latestEntry.getValue().getPose();
// currentHeading = currentPose.getRotation();
// }

// private void updateVelocity(double currentTime) {
// if (poseHistory.size() < 2) {
// currentVelocity = new Translation2d();
// return;
// }

// // Find poses within the velocity calculation window
// double windowStart = currentTime - velocityCalculationWindow.get();
// Map.Entry<Double, UncertainPose2d> startEntry =
// poseHistory.floorEntry(windowStart);

// // If no entry before window start, use the first entry
// if (startEntry == null) {
// startEntry = poseHistory.firstEntry();
// }

// // Get the latest pose
// Map.Entry<Double, UncertainPose2d> endEntry = poseHistory.lastEntry();

// // Calculate time difference
// double dt = endEntry.getKey() - startEntry.getKey();
// if (dt <= 0) {
// currentVelocity = new Translation2d();
// return;
// }

// // Calculate velocity
// Transform2d transform = new Transform2d(startEntry.getValue().getPose(),
// endEntry.getValue().getPose());
// currentVelocity = new Translation2d(transform.getX() / dt, transform.getY() /
// dt);
// }

// private void cleanupOldData() {
// double currentTime = Timer.getFPGATimestamp();
// double cutoffTime = currentTime - historyLength.get();

// // Remove all entries older than cutoff time
// poseHistory.headMap(cutoffTime).clear();
// }
// }{

// }
