package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A {@code PhotonCameraSimulator} aims to provide realistic simulations of a
 * {@code PhotonCamera} with artificially injected auglar errors and delays.
 */
public class PhotonCameraSimulator extends PhotonCamera {

	/**
	 * The {@code PhotonCamera} used by this {@code PhotonCameraSimulator}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code PhotonCameraSim} used by this {@code PhotonCameraSimulator}.
	 */
	private PhotonCameraSim m_cameraSim = null;

	/**
	 * The {@code VisionSystemSim} used by this {@code PhotonCameraSimulator}.
	 */
	private VisionSystemSim m_sim = null;

	/**
	 * The maximum artificial angular error to inject in radians.
	 */
	private double m_maxAngularError;

	/**
	 * The artificial delay to add in seconds.
	 */
	private double m_DelayInSeconds;

	/**
	 * The {@code PhotonPipelineResult}s that are buffered.
	 */
	private LinkedList<PhotonPipelineResult> m_buffered = new LinkedList<PhotonPipelineResult>();

	/**
	 * The {@code PhotonPipelineResult}s that are sufficiently delayed and thus are
	 * considered readable.
	 */
	private LinkedList<PhotonPipelineResult> m_unreadResults = new LinkedList<PhotonPipelineResult>();

	/**
	 * The latest {@code PhotonPipelineResult} that is sufficiently delayed and thus
	 * is considered readable.
	 */
	private PhotonPipelineResult m_latestResult = new PhotonPipelineResult();

	/**
	 * The {@code Pose2d} of the robot in simulation.
	 */
	private Pose2d m_pose;

	/**
	 * The previous {@code Pose2d} of the robot from the {@code DriveSubsystem}.
	 */
	private Pose2d m_previousOdometryPose = null;

	/**
	 * The error ratio in measurements for updating the odometry of the
	 * robot.
	 */
	private double m_measurmentErrorRatio;

	/**
	 * The {@code StructPublisher} for reporting the {@code Pose2d} of the
	 * robot in simulation.
	 */
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * The {@code StructPublisher} for reporting the {@code Pose2d} of the
	 * robot in simulation.
	 */
	private final StructPublisher<Pose3d> m_cameraPosePublisher;

	/**
	 * Constructs a {@code PhotonCameraSimulator}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PhotonCameraSimulator} (found in the PhotonVision UI)
	 * @param driveSubsystem the {@code DriveSubsystem} to be used by the
	 *        {@code PhotonCameraSimulator}
	 * @param initialPose the initial {@code Pose2d} of the robot in simulation
	 * @param measurmentErrorRatio the error ratio in measurements for
	 *        updating the odometry of the robot
	 * @param maxAngularErrorInDegrees the maximum artificial angular error to
	 *        inject in degrees
	 * @param delayInSeconds the artificial delay to add in seconds
	 */
	public PhotonCameraSimulator(String cameraName, DriveSubsystem driveSubsystem, Pose2d initialPose,
			double measurmentErrorRatio,
			double maxAngularErrorInDegrees,
			double delayInSeconds) {
		super(cameraName);
		m_driveSubsystem = driveSubsystem;
		m_pose = initialPose;
		m_measurmentErrorRatio = measurmentErrorRatio;
		m_maxAngularError = maxAngularErrorInDegrees * Math.PI / 180;
		m_DelayInSeconds = delayInSeconds;
		if (RobotBase.isSimulation()) {
			m_cameraSim = new PhotonCameraSim(this);
			m_cameraSim.enableProcessedStream(true);
			m_cameraSim.enableDrawWireframe(true);
			m_sim = new VisionSystemSim("sim");
			m_sim.addAprilTags(kFieldLayout);
			m_sim.addCamera(m_cameraSim, kRobotToCamera);
		} else {
			m_sim = null;
			m_cameraSim = null;
		}
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@Simulation", Pose2d.struct)
				.publish();
		m_cameraPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/CameraPose@Simulation", Pose3d.struct)
				.publish();
	}

	/**
	 * Returns the latest {@code PhotonPipelineResult} that is delayed
	 * sufficiently (by the specified delay in seconds) and thus considered
	 * readable.
	 * 
	 * @return the latest {@code PhotonPipelineResult} that is delayed
	 *         sufficiently (by the specified delay in seconds) and thus considered
	 *         readable
	 */
	@Override
	public PhotonPipelineResult getLatestResult() {
		update(super.getAllUnreadResults().stream().map(t -> distort(t)).toList());
		return m_latestResult;
	}

	/**
	 * Returns a list of {@code PhotonPipelineResult}s since the last call to
	 * getAllUnreadResults(). These {@code PhotonPipelineResult}s are delayed
	 * sufficiently (by the specified delay in seconds) and thus considered
	 * readable.
	 * 
	 * @return a list of {@code PhotonPipelineResult}s since the last call to
	 *         getAllUnreadResults(). These {@code PhotonPipelineResult}s are
	 *         delayed sufficiently (by the specified delay in seconds) and thus
	 *         considered readable
	 */
	@Override
	public List<PhotonPipelineResult> getAllUnreadResults() {
		update(super.getAllUnreadResults().stream().map(t -> distort(t)).toList());
		var r = m_unreadResults;
		m_unreadResults = new LinkedList<PhotonPipelineResult>();
		return r;
	}

	/**
	 * Randomly changes the specified value using the specified ratio.
	 * 
	 * @param x a value
	 * @param r the ratio by which the value can be increased or decreased
	 * @return the resulting value
	 */
	private double change(double x, double r) {
		return x * (1 - r + 2 * r * Math.random());
	}

	/**
	 * Updates this {@code PhotonCameraSimulator} using the specified
	 * {@code PhotonPipelineResult}s.
	 * 
	 * @param l a list of {@code PhotonPipelineResult}s
	 */
	private void update(List<PhotonPipelineResult> l) {
		m_buffered.addAll(l);
		if (m_buffered.size() > 0) {
			var timestamp = m_buffered.getLast().getTimestampSeconds() - this.m_DelayInSeconds;
			while (m_buffered.size() > 0 && m_buffered.getFirst().getTimestampSeconds() < timestamp) {
				var r = m_buffered.remove();
				m_latestResult = r;
				m_unreadResults.add(r);
				if (m_unreadResults.size() > 20)
					m_unreadResults.remove();
			}
		}
		var p = m_driveSubsystem.getPose();
		if (m_previousOdometryPose != null) {
			var t = p.minus(m_previousOdometryPose);
			t = new Transform2d(change(t.getX(), m_measurmentErrorRatio), change(t.getY(), m_measurmentErrorRatio),
					Rotation2d.fromDegrees(change(t.getRotation().getDegrees(), m_measurmentErrorRatio)));
			m_pose = m_pose.plus(t);
		}
		m_previousOdometryPose = p;
		m_sim.update(m_pose);
		m_posePublisher.set(m_pose);
		m_cameraPosePublisher.set(new Pose3d(m_pose).plus(kRobotToCamera));
	}

	/**
	 * Injects artificial angular errors in the specified
	 * {@code PhotonPipelineResult}.
	 * 
	 * @param r a {@code PhotonPipelineResult}
	 * @return the resulting {@code PhotonPipelineResult}
	 */
	private PhotonPipelineResult distort(PhotonPipelineResult r) {
		return new PhotonPipelineResult(r.metadata, r.targets.stream().map(t -> distort(t)).toList(), r.multitagResult);
	}

	/**
	 * Injects artificial angular errors in the specified
	 * {@code PhotonTrackedTarget}.
	 * 
	 * @param r a {@code PhotonTrackedTarget}
	 * @return the resulting {@code PhotonTrackedTarget}
	 */
	private PhotonTrackedTarget distort(PhotonTrackedTarget t) {
		t.bestCameraToTarget = t.bestCameraToTarget
				.plus(
						new Transform3d(0, 0, 0,
								new Rotation3d(randomAngularError(), randomAngularError(), randomAngularError())));
		return t;
	}

	/**
	 * Returns a random angular error in radians.
	 * 
	 * @return a random angular error in radians
	 */
	private double randomAngularError() {
		return Math.random() * 2 * m_maxAngularError - m_maxAngularError;
	}

}