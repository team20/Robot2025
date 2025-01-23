package frc.robot.commands;

import static frc.robot.Constants.*;

import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

/**
 * This {@code AlignCommand} supports various methods for aligning the robot.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AlignCommand extends Command {

	/**
	 * Constructs a {@code DriveCommand} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPose the {@code Pose2d} of the target
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		return new DriveCommand(driveSubsystem, () -> poseEstimationSubsystem.getPose(),
				() -> targetPose,
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for moving the robot toward the specified
	 * target position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimaitonSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPositionSupplier a {@code Supplier<Pose2d>} that provides the
	 *        target position.
	 *        This is used at the commencement of the {@code DriveCommand} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code DriveCommand})
	 * @param distanceToTarget the desired distance between the robot and the
	 *        target position
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static DriveCommand moveToward(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimaitonSubsystem,
			Supplier<Translation2d> targetPositionSupplier,
			double distanceToTarget,
			double distanceTolerance,
			double angleTolerance) {
		Supplier<Pose2d> poseSupplier = () -> poseEstimaitonSubsystem.getPose();
		return new DriveCommand(driveSubsystem, poseSupplier,
				() -> poseSupplier.get()
						.plus(transformationToward(targetPositionSupplier.get(), poseSupplier.get(), distanceToTarget)),
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param robotToTarget the {@code Pose2d} of the target relative to the
	 *        {@code Pose2d} of the robot for the {@code DriveCommand} to finish
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveToClosestTag(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Transform2d robotToTarget, double distanceTolerance, double angleTolerance) {
		return new DriveCommand(driveSubsystem, () -> poseEstimationSubsystem.getPose(),
				() -> kFieldLayout
						.getTagPose(closestTagID(poseEstimationSubsystem.getPose())).get()
						.toPose2d()
						.transformBy(robotToTarget),
				distanceTolerance, angleTolerance);
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target position and remain the specified distance away fron the target
	 * position.
	 * 
	 * @param targetPosition the target position whose x and y-coordinate values
	 *        are in meters
	 * @param distanceToTarget the desired distance in meters to the target
	 * @return the transformation needed for the robot to face toward the specified
	 *         target position and remain the specified distance away fron the
	 *         target position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public static Transform2d transformationToward(Translation2d targetPosition, Pose2d currentPose,
			double distanceToTarget) {
		Translation2d diff = targetPosition.minus(currentPose.getTranslation());
		if (diff.getNorm() == 0)
			return null;
		var targetPose = new Pose2d(
				currentPose.getTranslation().plus(diff.times(1 - distanceToTarget / diff.getNorm())),
				diff.getAngle());
		return targetPose.minus(currentPose);
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest to the specified
	 * {@code Pose2d}.
	 * 
	 * @param pose a {@code Pose2d}
	 * @return the ID of the {@code AprilTag} that is closest to the specified
	 *         {@code Pose2d}
	 */
	public static Integer closestTagID(Pose2d pose) {
		var s = kFieldLayout.getTags().stream()
				// only the tags facing toward the robot
				.filter(t -> Math.abs(t.pose.toPose2d().getRotation().minus(pose.getRotation()).getDegrees()) > 120)
				// only the tags near the center of the view
				.filter(t -> Math.abs(angularDisplacement(pose, t.pose.toPose2d()).getDegrees()) < 30)
				.map(t -> Map.entry(t.ID, translationalDisplacement(pose, t.pose.toPose2d()))) // displacement
				.filter(t -> t.getValue() > 0) // only the tags in front of the robot (not behind)
				.filter(t -> t.getValue() < 4); // only the tags that are within 3 meters away
		Optional<Entry<Integer, Double>> closest = s.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2);
		if (closest.isPresent()) {
			return closest.get().getKey();
		} else
			return null;
	}

	/**
	 * Calculates the angular displacement given the initial and last
	 * {@code Pose2d}s.
	 * 
	 * @param initial the initial {@code Pose2d}
	 * @param last the last {@code Pose2d}
	 * @return the angular displacement given the initial and last
	 *         {@code Pose2d}s
	 */
	public static Rotation2d angularDisplacement(Pose2d initial, Pose2d last) {
		var t = last.getTranslation().minus(initial.getTranslation());
		return t.getAngle().minus(initial.getRotation());
	}

	/**
	 * Calculates the translational displacement given the initial and last
	 * {@code Pose2d}s.
	 * 
	 * @param initial the initial {@code Pose2d}
	 * @param last the last {@code Pose2d}
	 * @return the translational displacement given the initial and last
	 *         {@code Pose2d}s
	 */
	public static double translationalDisplacement(Pose2d initial, Pose2d last) {
		var t = last.getTranslation().minus(initial.getTranslation());
		return Math.abs(t.getAngle().minus(initial.getRotation()).getDegrees()) > 90
				? -t.getNorm()
				: t.getNorm();
	}

}