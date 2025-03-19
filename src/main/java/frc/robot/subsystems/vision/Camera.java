package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;

public class Camera {
	private final PhotonCamera m_camera;
	/* Sim */
	private final PhotonCameraSim m_cameraSim;
	private final VisionSystemSim m_sim;

	// TODO measure robotToCamera transform - from center of robot to camera
	private final Transform3d robotToCamera;
	private Matrix<N3, N1> currentSTD; // Used to hold updated standard deviation values
	private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2025Reefscape);
	// Photon Pose Estimator information:n
	// https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/robot-pose-estimator.html
	private final PhotonPoseEstimator m_poseEstimator;
	private final StructPublisher<Pose2d> m_visionPosePublisher;

	// Creates a new Vision class.
	public Camera(String camera, Transform3d cameraTransform) {
		m_camera = new PhotonCamera(camera);
		robotToCamera = cameraTransform;
		m_poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				robotToCamera);
		m_visionPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/" + camera, Pose2d.struct).publish();
		// Initializes vision simulation system and camera, for more sim info
		// see https://docs.photonvision.org/en/v2025.1.1/docs/examples/poseest.html
		if (RobotBase.isSimulation()) {
			m_sim = new VisionSystemSim("sim");
			m_sim.addAprilTags(aprilTagFieldLayout);
			m_cameraSim = new PhotonCameraSim(m_camera);
			m_sim.addCamera(m_cameraSim, robotToCamera);
			m_cameraSim.enableProcessedStream(true);
			m_cameraSim.enableDrawWireframe(true);
		} else {
			m_sim = null;
			m_cameraSim = null;
		}
	}

	/**
	 * Calculates new standard deviations. This algorithm is a heuristic that
	 * creates dynamic standard deviations based on number of tags, estimation
	 * strategy, and distance from the tags.
	 * 
	 * Called in {@link getEstimatedGlobalPose}, so would recommend not calling this
	 * method independently.
	 * 
	 * Code references PhotonVision example:
	 * https://github.com/PhotonVision/photonvision/blob/v2025.1.1/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
	 * 
	 * @param estPose The estimated pose to guess standard deviations for.
	 * @param targets All targets in this camera frame.
	 */
	public void updateEstSTD(Optional<EstimatedRobotPose> estPose, List<PhotonTrackedTarget> targets) {
		// No pose input. Default to single-tag std devs. TODO Needs to be tuned.
		if (estPose.isEmpty()) {
			currentSTD = VecBuilder.fill(2, 2, 4);
		} else {// Pose present. Start running heuristic.
			var estSTD = currentSTD;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation - see how many tags we found,
			// and calculate an average-distance metric
			for (var t : targets) {
				var tagPose = m_poseEstimator.getFieldTags().getTagPose(t.getFiducialId());
				if (tagPose.isEmpty())
					continue;
				numTags++;
				avgDist += PhotonUtils
						.getDistanceToPose(estPose.get().estimatedPose.toPose2d(), tagPose.get().toPose2d());
			}

			// One or more tags visible, run the full heuristic.
			if (numTags != 0) {
				avgDist /= numTags;
				// Decrease std devs if multiple targets are visible.
				if (numTags > 1)
					estSTD = VecBuilder.fill(0.5, 0.5, 1);

				// Increase std devs based on (average) distance. TODO needs to be tuned.
				if (numTags == 1 && avgDist > 4)
					estSTD = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); // from example code
				else
					estSTD = estSTD.times(1 + (Math.pow(avgDist, 2) / 30)); // math taken from PhotonVision example code
				currentSTD = estSTD;
			}
		}
	}

	/**
	 * Returns the latest standard deviations of the estimated pose from {@link
	 * #getEstimatedGlobalPose()}, for use with {@link
	 * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
	 * SwerveDrivePoseEstimator}. This should
	 * only be used when there are targets visible.
	 */
	public Matrix<N3, N1> getEstimationStdDevs() {
		return currentSTD;
	}

	/**
	 * The latest estimated robot pose on the field from vision data. This may be
	 * empty. This should only be called once per loop.
	 * 
	 * Also includes updates for the standard deviations, which can (optionally) be
	 * retrieved with {@link getEstimationStdDevs}.
	 * 
	 * Code references PhotonVision example:
	 * https://github.com/PhotonVision/photonvision/blob/v2025.1.1/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
	 * 
	 * @param poseEstimator The pose estimator to update
	 */
	public void updatePoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
		Optional<EstimatedRobotPose> estimation = Optional.empty();
		for (var r : m_camera.getAllUnreadResults()) {
			// Loop through all pipeline results and update estimated pose.
			estimation = m_poseEstimator.update(r);
			updateEstSTD(estimation, r.getTargets());

			if (estimation.isPresent()) {
				var pose = estimation.get();
				// Update logged pose with estimation.
				m_visionPosePublisher.set(pose.estimatedPose.toPose2d());
				poseEstimator.addVisionMeasurement(
						pose.estimatedPose.toPose2d(), pose.timestampSeconds, getEstimationStdDevs());
			}
		}
	}
}
