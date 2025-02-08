package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@code PoseEstimationSubsystem} continuously estimates the {@code Pose2d}
 * of the robot on the field.
 */
public class PoseEstimationSubsystem extends SubsystemBase {

	/**
	 * The {@code PhotonCamera}s and {@code PhotonPoseEstimator}s used by this
	 * {@code PoseEstimationSubsystem}.
	 */
	private final Map<PhotonCamera, PhotonPoseEstimator> m_cameras = new HashMap<PhotonCamera, PhotonPoseEstimator>();

	/**
	 * The {@code PhotonCamera} used by this {@code PoseEstimationSubsystem}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The standard deviations of model states (smaller values will cause the Kalman
	 * filter to more trust the estimates).
	 */
	private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

	/**
	 * The standard deviations of the vision measurements (smaller values will cause
	 * the Kalman filter to more trust the measurements).
	 */
	private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

	/**
	 * The {@code SwerveDrivePoseEstimator} used by this
	 * {@code PoseEstimationSubsystem} to improve accuracy by injecting odometry
	 * data while addressing delays via Kalman filtering.
	 */
	private final SwerveDrivePoseEstimator m_poseEstimator;

	/**
	 * The {@code StructPublisher} for reporting the detected {@code Pose2d} of the
	 * robot.
	 */
	private final StructPublisher<Pose2d> m_detectedPosePublisher;

	/**
	 * The {@code StructPublisher} for reporting the estimated {@code Pose2d} of the
	 * robot.
	 */
	private final StructPublisher<Pose2d> m_estimatedPosePublisher;

	/**
	 * The {@code PIDController} for controlling the robot in the x and y
	 * dimensions in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	private PIDController m_controllerXY;

	/**
	 * The {@code PIDController} for controlling the robot in the yaw
	 * dimension in degrees (input: error in degrees, output: velocity in radians
	 * per second).
	 */
	private PIDController m_controllerYaw;

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 * 
	 * @param driveSubsystem {@code DriveSubsystem} to be used by the
	 *        {@code PoseEstimationSubsystem}
	 */
	public PoseEstimationSubsystem(DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_poseEstimator = new SwerveDrivePoseEstimator(
				driveSubsystem.kinematics(),
				driveSubsystem.getHeading(),
				driveSubsystem.getModulePositions(),
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs);
		m_detectedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonPoseEstimator", Pose2d.struct)
				.publish();
		m_estimatedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PoseEstimationSubsystem", Pose2d.struct)
				.publish();
		m_controllerXY = new PIDController(kDriveP, kDriveI, kDriveD);
		m_controllerYaw = new PIDController(kTurnP, kTurnI, kTurnD);
		m_controllerYaw.enableContinuousInput(-180, 180);
	}

	/**
	 * Adds the specified {@code PhotonCamera} to this
	 * {@code PoseEstimationSubsystem}.
	 * 
	 * @param cameraSim a {@code PhotonCamera}
	 * @param robotToCamera the {@code Transform3d} expressing the pose of the
	 *        camera relative to the pose of the robot.
	 * @return this {@code PoseEstimationSubsystem}
	 */
	public PoseEstimationSubsystem addCamera(PhotonCamera camera, Transform3d robotToCamera) {
		m_cameras.put(
				camera, new PhotonPoseEstimator(kFieldLayout,
						PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera));
		return this;
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		boolean firstCamera = true;
		for (var e : m_cameras.entrySet()) {
			var camera = e.getKey();
			var poseEstimator = e.getValue();
			for (var r : camera.getAllUnreadResults()) { // for every result r
				if (firstCamera || r.getTargets().size() > 1) {
					Optional<EstimatedRobotPose> p = poseEstimator.update(r);
					if (p.isPresent()) { // if successful
						EstimatedRobotPose v = p.get(); // get successfully estimated pose
						m_poseEstimator.addVisionMeasurement(v.estimatedPose.toPose2d(), v.timestampSeconds);
						m_detectedPosePublisher.set(v.estimatedPose.toPose2d());
					}
				}
			}
			firstCamera = false;
		}
		m_poseEstimator.update(m_driveSubsystem.getHeading(), m_driveSubsystem.getModulePositions());
		m_estimatedPosePublisher.set(m_poseEstimator.getEstimatedPosition());
	}

	/**
	 * Returns the most recent estimated {@code Pose2d} of the robot.
	 * 
	 * @return the most recent estimated {@code Pose2d} of the robot
	 */
	public Pose2d getEstimatedPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	/**
	 * Calculates the {@code ChassisSpeeds} to move the robot toward the closest
	 * {@code AprilTag}.
	 * 
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is aligned
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the calculated {@code ChassisSpeeds} to move the robot toward the
	 *         closest {@code AprilTag}
	 */
	public ChassisSpeeds chassisSpeedsTowardClosestTag(Transform2d robotToTag, double distanceThresholdInMeters) {
		var currentRobotPose = getEstimatedPose();
		var closestTagPose = closestTagPose(180, distanceThresholdInMeters);
		if (closestTagPose == null)
			return new ChassisSpeeds();
		var targetRobotPose = closestTagPose.plus(robotToTag);
		return chassisSpeeds(currentRobotPose, targetRobotPose, m_controllerXY, m_controllerYaw);
	}

	/**
	 * Calculates the {@code ChassisSpeeds} to move from the current {@code Pose2d}
	 * toward the target {@code Pose2d}.
	 * 
	 * @param currentPose the current {@code Pose2d}
	 * @param targetPose the target {@code Pose2d}
	 * @param contollerXY the {@code PIDController} for controlling the robot in the
	 *        x and y dimensions in meters (input: error in meters, output: velocity
	 *        in meters per second)
	 * @param controllerYaw the {@code PIDController} for controlling the robot in
	 *        the yaw dimension in degrees (input: error in degrees, output:
	 *        velocity in radians per second)
	 * @return the calculated {@code ChassisSpeeds} to move from the current
	 *         {@code Pose2d} toward the target {@code Pose2d}
	 */
	public static ChassisSpeeds chassisSpeeds(Pose2d currentPose, Pose2d targetPose, PIDController contollerXY,
			PIDController controllerYaw) {
		Translation2d current2target = targetPose.getTranslation()
				.minus(currentPose.getTranslation());
		double velocityX = 0, velocityY = 0;
		double distance = current2target.getNorm();
		if (distance > 0) {
			double speed = Math.abs(contollerXY.calculate(distance, 0));
			velocityX = speed * current2target.getAngle().getCos();
			velocityY = speed * current2target.getAngle().getSin();
		}
		double angularVelocityRadiansPerSecond = controllerYaw
				.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
		return new ChassisSpeeds(velocityX, velocityY, angularVelocityRadiansPerSecond);
	}

	/**
	 * Finds the {@code Pose2d} of the {@code AprilTag} that is closest to the robot
	 * ({@code null} if no such {@code AprilTag}).
	 * 
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the {@code Pose2d} of the {@code AprilTag} that is closest to the
	 *         robot ({@code null} if no such {@code AprilTag})
	 */
	public Pose2d closestTagPose(double angleOfCoverageInDegrees, double distanceThresholdInMeters) {
		var i = closestTagID(getEstimatedPose(), angleOfCoverageInDegrees, distanceThresholdInMeters);
		return i == null ? null : kFieldLayout.getTagPose(i).get().toPose2d();
	}

	/**
	 * Finds the {@code Pose2d} of the {@code AprilTag} that is closest (in terms of
	 * angular displacement) to the robot when the specified {@code Translation2d}
	 * is applied to the robot.
	 * 
	 * @param translation a {@code Translation2d}
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the {@code Pose2d} of the {@code AprilTag} that is closest (in terms
	 *         of angular displacement) to the robot when the specified
	 *         {@code Translation2d} is applied to the robot
	 */
	public Pose2d closestTagPose(Translation2d translation, double distanceThresholdInMeters) {
		var pose = getEstimatedPose();
		if (translation.getNorm() > 0)
			pose = new Pose2d(pose.getTranslation(), translation.getAngle());
		var i = closestTagID(pose, distanceThresholdInMeters);
		return i == null ? null : kFieldLayout.getTagPose(i).get().toPose2d();
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest to the specified
	 * {@code Pose2d} ({@code null} if no such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the ID of the {@code AprilTag} that is closest to the specified
	 *         {@code Pose2d} ({@code null} if no such {@code AprilTag})
	 */
	public static Integer closestTagID(Pose2d pose, double angleOfCoverageInDegrees, double distanceThresholdInMeters) {
		var s = kFieldLayout.getTags().stream()
				// consider only the tags facing toward the robot
				.filter(
						t -> Math.abs(
								t.pose.getTranslation().toTranslation2d().minus(pose.getTranslation()).getAngle()
										.minus(t.pose.toPose2d().getRotation()).getDegrees()) > 90)
				.filter( // consider only the tags within the angle of coverage
						t -> Math.abs(
								angularDisplacement(pose, t.pose.toPose2d()).getDegrees()) < angleOfCoverageInDegrees)
				.map(t -> Map.entry(t.ID, Math.abs(translationalDisplacement(pose, t.pose.toPose2d())))) // distance
				.filter(t -> t.getValue() < distanceThresholdInMeters); // only tags sufficently close
		Optional<Entry<Integer, Double>> closest = s.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2);
		if (closest.isPresent()) {
			return closest.get().getKey();
		} else
			return null;
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest (in terms of
	 * angular displacement) to the specified {@code Pose2d} ({@code null} if no
	 * such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the ID of the {@code AprilTag} that is closest (in terms of angular
	 *         displacement) to the specified {@code Pose2d} ({@code null} if no
	 *         such {@code AprilTag})
	 */
	public static Integer closestTagID(Pose2d pose, double distanceThresholdInMeters) {
		var s = kFieldLayout.getTags().stream()
				// consider only the tags facing toward the robot
				.filter(
						t -> Math.abs(
								t.pose.getTranslation().toTranslation2d().minus(pose.getTranslation()).getAngle()
										.minus(t.pose.toPose2d().getRotation()).getDegrees()) > 90)
				.filter(t -> Math.abs(translationalDisplacement(pose, t.pose.toPose2d())) < distanceThresholdInMeters)
				// only tags sufficently close
				.map(t -> Map.entry(t.ID, Math.abs(angularDisplacement(pose, t.pose.toPose2d()).getDegrees())));
		Optional<Entry<Integer, Double>> closest = s.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2);
		if (closest.isPresent()) {
			return closest.get().getKey();
		} else
			return null;
	}

	/**
	 * Calculates the angular displacement from the estimated {@code Pose2d} of the
	 * robot to the specified {@code AprilTag}.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @return the angular displacement from the estimated {@code Pose2d} of the
	 *         robot to the specified {@code AprilTag}
	 */
	public Rotation2d angularDisplacement(int tagID) {
		try {
			return angularDisplacement(this.getEstimatedPose(), kFieldLayout.getTagPose(tagID).get().toPose2d());
		} catch (Exception e) {
			return null;
		}
	}

	/**
	 * Calculates the translational displacement from the initial {@code Pose2d} to
	 * the last {@code Pose2d}.
	 * 
	 * @param initial the initial {@code Pose2d}
	 * @param last the last {@code Pose2d}
	 * @return the translational displacement from the initial {@code Pose2d} to
	 *         the last {@code Pose2d}
	 */
	public static double translationalDisplacement(Pose2d initial, Pose2d last) {
		var t = last.getTranslation().minus(initial.getTranslation());
		return Math.abs(t.getAngle().minus(initial.getRotation()).getDegrees()) > 90 ? -t.getNorm() : t.getNorm();
	}

	/**
	 * Calculates the angular displacement from the initial {@code Pose2d} to
	 * the last {@code Pose2d}.
	 * 
	 * @param initial the initial {@code Pose2d}
	 * @param last the last {@code Pose2d}
	 * @return the angular displacement from the initial {@code Pose2d} to
	 *         the last {@code Pose2d}
	 */
	public static Rotation2d angularDisplacement(Pose2d initial, Pose2d last) {
		var t = last.getTranslation().minus(initial.getTranslation());
		return t.getAngle().minus(initial.getRotation());
	}

	/**
	 * Constructs a {@code Pose2d}.
	 * 
	 * @param x the x component of the {@code Pose2d}
	 * @param y the y component of the {@code Pose2d}
	 * @param yawInDegrees the rotational component of the {@code Pose2d} in degrees
	 * @return the constructed {@code Pose2d}
	 */
	public static Pose2d pose(double x, double y, double yawInDegrees) {
		return new Pose2d(x, y, Rotation2d.fromDegrees(yawInDegrees));
	}

}