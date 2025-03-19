package frc.robot.subsystems;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;

public class Vision {
	// Declare class variables
	private List<PhotonPipelineResult> m_results; // Used to hold results from camera stream in one place
	private final PhotonCamera m_camera = new PhotonCamera("Cool camera");;
	private final PhotonCameraSim m_cameraSim;
	private final VisionSystemSim m_sim;
	// TODO measure robotToCamera transform - from center of robot to camera
	private final Transform3d robotToCamera = new Transform3d(0.5, 0.5, 0.5, new Rotation3d());
	private Matrix<N3, N1> currentSTD; // Used to hold updated standard deviation values
	private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2025Reefscape);
	// Photon Pose Estimator information:
	// https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/robot-pose-estimator.html
	private final PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(
			aprilTagFieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			robotToCamera);
	private final StructPublisher<Pose2d> m_visionPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("/SmartDashboard/VisionPose", Pose2d.struct).publish();

	// Creates a new Vision class.
	public Vision() {
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
	 * The list of pipeline results sent by PhotonVision since the last call to
	 * {@link getAllUnreadResults}. Calling this function clears the internal FIFO
	 * queue,
	 * and multiple calls to {@link getAllUnreadResults} will return different
	 * (potentially empty) result arrays.
	 * 
	 * Be careful to call this exactly ONCE per loop of your robot code!
	 * FIFO depth is limited to 20 changes, so make sure to
	 * call this frequently enough to avoid old results being discarded, too!
	 * https://javadocs.photonvision.org/org/photonvision/PhotonCamera.html#getAllUnreadResults()
	 */
	public void refreshResults() {
		m_results = m_camera.getAllUnreadResults();
	}

	/**
	 * Gets the unread pipeline results from the camera's FIFO queue.
	 * 
	 * @return The last 20 results from the camera
	 */
	public List<PhotonPipelineResult> getResults() {
		return m_results;
	}

	/**
	 * Gets the latest pipeline result from the camera's FIFO queue.
	 * 
	 * @return The latest result from the camera
	 */
	public PhotonPipelineResult getLatestResult() {
		return m_camera.getLatestResult();
	}

	/**
	 * Gets the best target seen by the camera from a pipeline result.
	 * Target sorting is determined by the PhotonVision UI,
	 * which can be changed under Contours on the UI.
	 * See Contour Grouping and Sorting:
	 * https://docs.photonvision.org/en/latest/docs/reflectiveAndShape/contour-filtering.html
	 * 
	 * @param r The pipeline result
	 * @return The best target seen by the camera
	 */
	public PhotonTrackedTarget getBestTarget(PhotonPipelineResult r) {
		return r.getBestTarget();
	}

	/**
	 * Gets the field-relative pose of the target.
	 * Series of methods below are from:
	 * https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/using-target-data.html
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The target's pose
	 */
	public Pose3d getTargetPose(PhotonTrackedTarget target) {
		return aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
	}

	/**
	 * Gets the field-relative pose of the robot.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The robot's pose
	 */
	public Pose3d getVisionPose(PhotonTrackedTarget target) {
		if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
			return PhotonUtils.estimateFieldToRobotAprilTag(
					target.getBestCameraToTarget(),
					getTargetPose(target),
					new Transform3d(-0.5, -0.5, -0.5, new Rotation3d())); // TODO fix camera to robot
		} else {
			return null;
		}
	}

	/**
	 * Gets the robot's distance to the target.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The distance to the target (in meters)
	 */
	public double getDistanceToTarget(PhotonTrackedTarget target) {
		return PhotonUtils.getDistanceToPose(
				getVisionPose(target).toPose2d(),
				getTargetPose(target).toPose2d());
	}

	/**
	 * Gets the robot's transform to the target.
	 * Translation and angle to target can be retrieved from transform.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The transform (x,y,theta) to the target
	 */
	public Transform3d getTransformToTarget(PhotonTrackedTarget target) {
		return robotToCamera.plus(target.getBestCameraToTarget());
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
			Pose3d tagPose;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation - see how many tags we found,
			// and calculate an average-distance metric
			for (var t : targets) {
				try {
					tagPose = getTargetPose(t);
				} catch (NoSuchElementException e) {
					continue;
				}
				numTags++;
				avgDist += PhotonUtils.getDistanceToPose(estPose.get().estimatedPose.toPose2d(), tagPose.toPose2d());
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
	 * Returns the latest standard deviations of the estimated pose from
	 * {@link getEstimatedGlobalPose}.
	 * This should only be used when there are targets visible.
	 */
	public Matrix<N3, N1> getCurrentSTD() {
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
	 * @param results The list of pipeline results from the camera stream.
	 * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
	 *         timestamp, and targets
	 *         used for estimation.
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(List<PhotonPipelineResult> results) {
		Optional<EstimatedRobotPose> estimation = Optional.empty();
		for (var r : results) {
			// Loop through all pipeline results and update estimated pose.
			estimation = m_poseEstimator.update(r);
			updateEstSTD(estimation, r.getTargets());
		}
		if (estimation.isPresent()) {
			// Update logged pose with estimation.
			m_visionPosePublisher.set(estimation.get().estimatedPose.toPose2d());
		}
		return estimation;
	}

	/**
	 * Update simulated vision pose with given robot pose.
	 */
	public void updateVisionSim(Pose2d botPose) {
		m_sim.update(botPose);
	}
}