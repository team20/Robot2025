//TODO: Make this into a subsystem inline

package frc.robot.subsystems;

import java.util.Map;

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
 * This {@code PhotonPoseEstimationSubsystem} uses a {@code PhotonCamera} and a
 * {@code PhotonPoseEstimator} to keep track of the current {@code Pose2d} of
 * the robot.
 */
public class PhotonPoseEstimationSubsystem extends SubsystemBase {

	/**
	 * The {@code PhotonCamera} used by this {@code PhotonPoseEstimationSubsystem}.
	 */
	private PhotonCamera m_camera;

	/**
	 * The {@code PhotonPoseEstimator} used by this
	 * {@code PhotonPoseEstimationSubsystem}.
	 */
	private PhotonPoseEstimator m_poseEstimator;

	/**
	 * The latest estimated {@code Pose2d} of the robot.
	 */
	private Pose2d m_pose;

	/**
	 * The {@code AprilTagFieldLayout} used by this
	 * {@code PhotonPoseEstimationSubsystem}.
	 */
	private AprilTagFieldLayout m_fieldLayout;

	/**
	 * The {@code StructPublisher} for reporting the current {@code Pose2d} of
	 * the robot.
	 */
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * Constructs a {@code PhotonPoseEstimationSubsystem}.
	 */
	public PhotonPoseEstimationSubsystem() {
		m_camera = new PhotonCamera("Cool camera");
		// TODO enter the correct x, y, and z-coordinate values
		Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
		m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
		m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonPoseEstimationSubsystem", Pose2d.struct)
				.publish();
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		for (var r : m_camera.getAllUnreadResults()) {
			var e = m_poseEstimator.update(r);
			if (e.isPresent()) {
				m_pose = e.get().estimatedPose.toPose2d();
				m_posePublisher.set(m_pose);
			}
		}
	}

	/**
	 * Returns the estimated {@code Pose2d} of the robot.
	 * 
	 * @return the estimated {@code Pose2d} of the robot
	 */
	public Pose2d getPose() {
		return m_pose;
	}

	/**
	 * Returns the {@code AprilTagFieldLayout} used by this
	 * {@code PhotonPoseEstimationSubsystem}.
	 * 
	 * @return the {@code AprilTagFieldLayout} used by this
	 *         {@code PhotonPoseEstimationSubsystem}
	 */
	public AprilTagFieldLayout getFieldLayout() {
		return m_fieldLayout;
	}

	/**
	 * Returns the {@code AprilTag} closest to the robot.
	 * 
	 * @return the {@code AprilTag} closest to the robot
	 */
	public AprilTag getClosestTag() {
		return m_pose == null ? null
				: m_fieldLayout.getTags().stream()
						.map(t -> Map.entry(t, PhotonUtils.getDistanceToPose(m_pose, t.pose.toPose2d())))
						.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2).get().getKey();
	}

}