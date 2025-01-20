package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.RobotBase;

public class PhotonCameraSimulator extends PhotonCamera {

	/**
	 * The {@code PhotonCamera} used by this {@code PoseEstimationSubsystem}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code PhotonCameraSim} used by this {@code PoseEstimationSubsystem}.
	 */
	private PhotonCameraSim m_cameraSim = null;

	/**
	 * The {@code VisionSystemSim} used by this {@code PoseEstimationSubsystem}.
	 */
	private VisionSystemSim m_sim = null;

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PoseEstimationSubsystem} (found in the PhotonVision UI)
	 * @param driveSubsystem {@code DriveSubsystem} to be used by the
	 *        {@code PoseEstimationSubsystem}
	 */
	public PhotonCameraSimulator(String cameraName, DriveSubsystem driveSubsystem) {
		super(cameraName);
		this.m_driveSubsystem = driveSubsystem;
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

	@Override
	public PhotonPipelineResult getLatestResult() {
		var r = super.getLatestResult();
		m_sim.update(m_driveSubsystem.getPose());
		return r;
	}

	@Override
	public List<PhotonPipelineResult> getAllUnreadResults() {
		var r = super.getAllUnreadResults();
		m_sim.update(m_driveSubsystem.getPose());
		return r;
	}

}