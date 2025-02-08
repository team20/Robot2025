package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * A {@code PhotonCameraSimulator} aims to provide realistic simulations of a
 * {@code PhotonCamera} with artificially injected auglar errors and delays.
 */
public class PhotonCameraSimulator extends PhotonCamera {

	/**
	 * The {@code PhotonCameraSim} used by this {@code PhotonCameraSimulator}.
	 */
	private PhotonCameraSim m_cameraSim = null;

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
	 * The {@code StructPublisher} for reporting the {@code Pose2d} of the
	 * robot in simulation.
	 */
	private final StructPublisher<Pose3d> m_cameraPosePublisher;

	/**
	 * The {@code VisionSimulator} used by this {@code PhotonCameraSimulator}.
	 */
	private VisionSimulator m_visionSimulator;

	/**
	 * Constructs a {@code PhotonCameraSimulator}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PhotonCameraSimulator} (found in the PhotonVision UI)
	 * @param robotToCamera the {@code Transform3d} expressing the pose of the
	 *        camera relative to the pose of the robot.
	 * @param visionSimulator the {@code VisionSimulator} to be used by the
	 *        {@code PhotonCameraSimulator}
	 * @param initialPose the initial {@code Pose2d} of the robot in simulation
	 * @param measurmentErrorRatio the error ratio in measurements for
	 *        updating the odometry of the robot
	 * @param maxAngularErrorInDegrees the maximum artificial angular error to
	 *        inject in degrees
	 * @param delayInSeconds the artificial delay to add in seconds
	 */
	public PhotonCameraSimulator(String cameraName, Transform3d robotToCamera,
			VisionSimulator visionSimulator,
			double maxAngularErrorInDegrees,
			double delayInSeconds) {
		super(cameraName);
		m_visionSimulator = visionSimulator;
		m_maxAngularError = maxAngularErrorInDegrees * Math.PI / 180;
		m_DelayInSeconds = delayInSeconds;
		m_cameraSim = new PhotonCameraSim(this);
		m_cameraSim.enableProcessedStream(true);
		m_cameraSim.enableDrawWireframe(true);
		visionSimulator.addCamera(m_cameraSim, robotToCamera);
		m_cameraPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/" + cameraName + "@Simulation", Pose3d.struct)
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
		m_cameraPosePublisher.set(m_visionSimulator.getCameraPose(m_cameraSim).get());
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