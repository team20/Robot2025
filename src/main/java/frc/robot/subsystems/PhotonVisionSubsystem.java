package frc.robot.subsystems;

import java.util.Map;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This {@code PhotonVisionSubsystem} uses a {@code PhotonCamera} and a
 * {@code PhotonPoseEstimator} to keep track of the current {@code Pose2d} of
 * the robot.
 */
public class PhotonVisionSubsystem extends SubsystemBase {

	/**
	 * The {@code PhotonCamera} used by this {@code PhotonVisionSubsystem}.
	 */
	private PhotonCamera m_camera;

	/**
	 * The {@code PhotonPoseEstimator} used by this
	 * {@code PhotonVisionSubsystem}.
	 */
	private PhotonPoseEstimator m_poseEstimator;

	/**
	 * The latest {@code EstimatedRobotPose} of the robot.
	 */
	private EstimatedRobotPose m_pose;

	/**
	 * The {@code AprilTagFieldLayout} used by the {@code PhotonVisionSubsystem}.
	 */
	public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	/**
	 * The {@code StructPublisher} for reporting the current {@code Pose2d} of
	 * the robot.
	 */
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * Constructs a {@code PhotonVisionSubsystem}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PhotonVisionSubsystem} (found in the PhotonVision UI).
	 */
	public PhotonVisionSubsystem(String cameraName) { // "Cool camera"
		m_camera = new PhotonCamera(cameraName);
		// TODO enter the correct x, y, and z-coordinate values
		Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
		// TODO choose the best PoseStrategy
		m_poseEstimator = new PhotonPoseEstimator(fieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonVisionSubsystem", Pose2d.struct)
				.publish();
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		for (var r : m_camera.getAllUnreadResults()) {
			var e = m_poseEstimator.update(r);
			if (e.isPresent())
				m_pose = e.get();
		}
		m_posePublisher.set(m_pose == null ? null : m_pose.estimatedPose.toPose2d());
	}

	/**
	 * Returns the most recent {@code EstimatedRobotPose} of the robot (may be
	 * outdated if recent pose detections failed consecutively).
	 * 
	 * @return the most recent {@code EstimatedRobotPose} of the robot (may be
	 *         outdated if recent pose detections failed consecutively)
	 */
	public EstimatedRobotPose getPose() {
		return m_pose;
	}

	/**
	 * Returns the {@code AprilTag} closest to the robot.
	 * 
	 * @return the {@code AprilTag} closest to the robot
	 */
	public AprilTag getClosestTag() {
		return getClosestTag(m_pose == null ? null : m_pose.estimatedPose.toPose2d());
	}

	/**
	 * Returns the {@code AprilTag} closest to the specified {@code Pose2d}.
	 * 
	 * @param pose a {@code Pose2d}
	 * @return the {@code AprilTag} closest to the specified {@code Pose2d}
	 */
	public static AprilTag getClosestTag(Pose2d pose) {
		return pose == null ? null
				: fieldLayout.getTags().stream()
						.map(t -> Map.entry(t, PhotonUtils.getDistanceToPose(pose, t.pose.toPose2d())))
						.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2).get().getKey();
	}

}