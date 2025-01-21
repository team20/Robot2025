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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
	 * Constructs a {@code PhotonCameraSimulator}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PhotonCameraSimulator} (found in the PhotonVision UI)
	 * @param driveSubsystem {@code DriveSubsystem} to be used by the
	 *        {@code PhotonCameraSimulator}
	 * @param maxAngularErrorInDegrees the maximum artificial angular error to
	 *        inject in degrees
	 * @param delayInSeconds the artificial delay to add in seconds
	 */
	public PhotonCameraSimulator(String cameraName, DriveSubsystem driveSubsystem, double maxAngularErrorInDegrees,
			double delayInSeconds) {
		super(cameraName);
		this.m_driveSubsystem = driveSubsystem;
		this.m_maxAngularError = maxAngularErrorInDegrees * Math.PI / 180;
		this.m_DelayInSeconds = delayInSeconds;
		if (RobotBase.isSimulation()) {
			m_sim = new VisionSystemSim("sim");
			m_sim.addAprilTags(kFieldLayout);
			m_cameraSim = new PhotonCameraSim(this);
			m_sim.addCamera(m_cameraSim, kRobotToCamera);
			m_cameraSim.enableProcessedStream(true);
			m_cameraSim.enableDrawWireframe(true);
		} else {
			m_sim = null;
			m_cameraSim = null;
		}
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
		process(super.getAllUnreadResults().stream().map(t -> distort(t)).toList());
		var r = m_latestResult;
		m_sim.update(m_driveSubsystem.getPose());
		return r;
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
		process(super.getAllUnreadResults().stream().map(t -> distort(t)).toList());
		var r = m_unreadResults;
		m_unreadResults = new LinkedList<PhotonPipelineResult>();
		m_sim.update(m_driveSubsystem.getPose());
		return r;
	}

	/**
	 * Processes the specified {@code PhotonPipelineResult}s.
	 * 
	 * @param l a list of {@code PhotonPipelineResult}s
	 */
	private void process(List<PhotonPipelineResult> l) {
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
	}

	/**
	 * Injects artificial angular errors in the specified
	 * {@code PhotonPipelineResult}.
	 * 
	 * @param r a {@code PhotonPipelineResult}
	 * @return the resulting {@code PhotonPipelineResult}
	 */
	private PhotonPipelineResult distort(PhotonPipelineResult r) {
		r.metadata.captureTimestampMicros -= this.m_DelayInSeconds * 1e6;
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