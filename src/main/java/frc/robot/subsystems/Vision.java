//TODO: Make this into a subsystem inline

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
	PhotonCamera camera;
	AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
	PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
			aprilTagFieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			new Transform3d(0.5, 0.5, 0.5, new Rotation3d())); // TODO change z for tag height

	// Creates a new Vision class.
	public Vision() {
		camera = new PhotonCamera("Cool camera");
	}

	/**
	 * Gets the ID of the best target seen by the camera.
	 * 
	 * @return The Fiducial ID of the viewed April Tag.
	 */
	public int getTargetId() {
		List<PhotonPipelineResult> results = camera.getAllUnreadResults();

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

	/**
	 * Identifies the best target seen by the camera
	 * (should be run periodically for up-to-date results).
	 * 
	 * @return The best target (April Tag)
	 */
	public PhotonTrackedTarget getBestTarget() {
		return camera.getAllUnreadResults().get(0).getBestTarget();
	}

	/**
	 * Gets the field-relative pose of the target.
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
		// TODO Will this pose (haha) a problem if the robot pose and tag pose aren't
		// retrieved at the same moment?
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
}