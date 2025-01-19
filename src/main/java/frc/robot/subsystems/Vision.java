//TODO: Make this into a subsystem inline

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
	private List<PhotonPipelineResult> m_results;
	private final PhotonCamera m_camera = new PhotonCamera("Cool camera");;
	private final PhotonCameraSim m_cameraSim;
	private final VisionSystemSim m_sim;
	private final Transform3d robotToCamera = new Transform3d(0.5, 0.5, 0.5, new Rotation3d());
	Matrix<N3, N1> currentSTD;
	private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2025Reefscape);
	private final PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(
			aprilTagFieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			robotToCamera); // TODO change z for tag height

	// Creates a new Vision class.
	public Vision() {
		// Initializes vision simulation system and camera
		if (RobotBase.isSimulation()) {
			m_sim = new VisionSystemSim("sim");
			m_sim.addAprilTags(aprilTagFieldLayout);
			m_cameraSim = new PhotonCameraSim(m_camera);
			m_sim.addCamera(m_cameraSim, robotToCamera);
			m_cameraSim.enableDrawWireframe(true); // check if this affects robot
		} else {
			m_sim = null;
			m_cameraSim = null;
		}

	}

	/**
	 * Gets the ID of the best target seen by the camera.
	 * 
	 * @return The Fiducial ID of the viewed April Tag.
	 */
	public int getTargetId() {
		List<PhotonPipelineResult> results = this.m_results;

		if (results.size() < 1) {
			return -1;
		}

		PhotonPipelineResult result = results.get(0);

		SmartDashboard.putBoolean("Has Targets", result.hasTargets());

		if (result.hasTargets()) {
			return result.getBestTarget().getFiducialId();
		}

		else {
			return -1;
		}
	}

	public List<PhotonPipelineResult> getTargets() {
		m_results = m_camera.getAllUnreadResults();
		return m_results;
	}

	/**
	 * Identifies the best target seen by the camera
	 * (should be run periodically for up-to-date results).
	 * 
	 * @return The best target (April Tag)
	 */
	public PhotonTrackedTarget getBestTarget() {
		return this.m_results.get(0).getBestTarget();
	}

	/**
	 * Gets the field-relative pose of the target.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The target's pose
	 */
	public Pose3d getTargetPose(PhotonTrackedTarget target) {
		target = (target == null) ? null : target; // TODO check for null in frontend instead
		return aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
	}

	/**
	 * Gets the field-relative pose of the robot.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The robot's pose
	 */
	public Pose3d getVisionPose(PhotonTrackedTarget target) {
		target = (target == null) ? null : target;
		if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
			return PhotonUtils.estimateFieldToRobotAprilTag(
					target.getBestCameraToTarget(),
					getTargetPose(target),
					new Transform3d(-0.5, -0.5, -0.5, new Rotation3d())); // TODO fix camera to robot
		} else
			return null;
	}

	/**
	 * Gets the robot's distance to the target.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The distance to the target
	 */
	public double getDistanceToTarget(PhotonTrackedTarget target) {
		return PhotonUtils.getDistanceToPose(
				getVisionPose(target).toPose2d(),
				getTargetPose(target).toPose2d());
	}

	/**
	 * Gets the robot's translation to the target.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The translation (x,y) to the target
	 */
	public Translation2d getTranslationToTarget(PhotonTrackedTarget target) {
		return PhotonUtils.estimateCameraToTargetTranslation(
				getDistanceToTarget(target),
				Rotation2d.fromDegrees(-target.getYaw())); // negated to switch from CW + to CCW +
	}

	/**
	 * Gets the robot's rotation (angle/yaw) to the target.
	 * 
	 * @param target The target (April Tag) viewed by the camera.
	 * @return The rotation to the target
	 */
	public Rotation2d getYawToPose(PhotonTrackedTarget target) {
		return PhotonUtils.getYawToPose(
				getVisionPose(target).toPose2d(),
				getTargetPose(target).toPose2d());
	}

	public void updateEstSTD(Optional<EstimatedRobotPose> estPose, List<PhotonTrackedTarget> targets) {
		currentSTD = VecBuilder.fill(2, 2, 4); // TODO taken from PhotonVision example code
		if (!estPose.isEmpty()) {
			var estSTD = currentSTD;
			Pose3d tagPose;
			int numTags = 0;
			double avgDist = 0;
			for (var t : targets) {
				try {
					tagPose = getTargetPose(t);
				} catch (NoSuchElementException e) {
					continue;
				}
				numTags++;
				avgDist += PhotonUtils.getDistanceToPose(estPose.get().estimatedPose.toPose2d(), tagPose.toPose2d());
				// TODO turn into a method?
			}

			if (numTags != 0) {
				avgDist /= numTags;
				if (numTags > 1)
					estSTD = VecBuilder.fill(0.5, 0.5, 1); // TODO taken from PhotonVision example code
				if (numTags == 1 && avgDist > 4)
					estSTD = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); // from example code
				else
					estSTD = estSTD.times(1 + (Math.pow(avgDist, 2) / 30)); // math taken from PhotonVision example code
				currentSTD = estSTD;
			}
		}
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		Optional<EstimatedRobotPose> estimation = Optional.empty();
		for (var r : m_camera.getAllUnreadResults()) {
			estimation = m_poseEstimator.update(r);
			updateEstSTD(estimation, r.getTargets());

			if (RobotBase.isSimulation()) {
				estimation.ifPresentOrElse(
						est -> m_sim.getDebugField().getObject("VisionEstimation")
								.setPose(est.estimatedPose.toPose2d()),
						() -> m_sim.getDebugField().getObject("VisionEstimation").setPoses());
			}
		}
		return estimation;
	}

	/**
	 * Gets the robot's rotation (angle/yaw) to the target.
	 * 
	 * @param botPose The tar
	 */
	public void updateVisionSim(Pose2d botPose) {
		m_sim.update(botPose);
	}
}