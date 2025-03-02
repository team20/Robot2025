package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code PathDriveCommand} aims to maneuver the robot to certain
 * {@code Pose2d}s. It utilizes two {@code PIDController}s to precisely
 * control the robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PathDriveCommand extends DriveCommand {

	/**
	 * The distance error in meters which is tolerable.
	 */
	protected double m_distanceTolerance;

	/**
	 * The angle error in radians which is tolerable.
	 */
	protected double m_angleTolerance;

	/**
	 * The distance error in meters which is tolerable for intermeidate
	 * target {@code Pose2d}s.
	 */
	protected double m_intermediateDistanceTolerance;

	/**
	 * The angle error in radians which is tolerable for intermeidate
	 * target {@code Pose2d}s.
	 */
	protected double m_intermediateAngleTolerance;

	/**
	 * Constructs a new {@code PathDriveCommand} whose purpose is to move
	 * the robot to certain {@code Pose2d}s.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param intermediateDistanceTolerance the distance error in meters which is
	 *        tolerable for intermeidate target {@code Pose2d}s
	 * @param intermediateAngleToleranceInDegrees the angle error in degrees which
	 *        is tolerable for intermeidate target {@code Pose2d}s
	 * @param targetPoses {@code Pose2d}s to which the robot should move
	 */
	public PathDriveCommand(DriveSubsystem driveSubsystem, double distanceTolerance, double angleToleranceInDegrees,
			double intermediateDistanceTolerance, double intermediateAngleToleranceInDegrees, Pose2d... targetPoses) {
		this(driveSubsystem, distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
				intermediateAngleToleranceInDegrees,
				Arrays.stream(targetPoses).map(p -> (Supplier<Pose2d>) (() -> p)).toList());
	}

	/**
	 * Constructs a new {@code PathDriveCommand} whose purpose is to move
	 * the robot to certain {@code Pose2d}s.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param intermediateDistanceTolerance the distance error in meters which is
	 *        tolerable for intermeidate target {@code Pose2d}s
	 * @param intermediateAngleToleranceInDegrees the angle error in degrees which
	 *        is tolerable for intermeidate target {@code Pose2d}s
	 * @param targetPoseSuppliers {@code Supplier<Pose2d>}s that provide the
	 *        {@code Pose2d}s to which the robot should move
	 */
	public PathDriveCommand(DriveSubsystem driveSubsystem, double distanceTolerance, double angleToleranceInDegrees,
			double intermediateDistanceTolerance, double intermediateAngleToleranceInDegrees,
			List<Supplier<Pose2d>> targetPoseSuppliers) {
		super(driveSubsystem, distanceTolerance, angleToleranceInDegrees,
				new IterativeTargetPoseSupplier(targetPoseSuppliers));
		m_distanceTolerance = distanceTolerance;
		m_angleTolerance = Math.toRadians(angleToleranceInDegrees);
		m_intermediateDistanceTolerance = intermediateDistanceTolerance;
		m_intermediateAngleTolerance = Math.toRadians(intermediateAngleToleranceInDegrees);
	}

	/**
	 * Is invoked at the commencement of this {@code PathDriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code PathDriveCommand}).
	 */
	@Override
	public void initialize() {
		targetPoseSupplier().reset();
		moveToNextTargetPose();
		Pose2d pose = m_driveSubsystem.getPose();
		m_controllerXY.reset(m_targetPose.minus(pose).getTranslation().getNorm());
		m_controllerYaw.reset(pose.getRotation().getRadians());
	}

	/**
	 * Moves to the next target {@code Pose2d}.
	 */
	protected void moveToNextTargetPose() {
		Pose2d pose = m_driveSubsystem.getPose();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
			e.printStackTrace();
			m_targetPose = pose;
		}
		if (targetPoseSupplier().hasNext()) {
			m_controllerXY.setTolerance(m_intermediateDistanceTolerance);
			m_controllerYaw.setTolerance(m_intermediateAngleTolerance);
		} else {
			m_controllerXY.setTolerance(m_distanceTolerance);
			m_controllerYaw.setTolerance(m_angleTolerance);
		}
	}

	/**
	 * Determines whether or not this {@code PathDriveCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code PathDriveCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		if (at(m_targetPose)) {
			if (targetPoseSupplier().hasNext())
				moveToNextTargetPose();
			else
				return true;
		}
		return false;
	}

	/**
	 * Determines whether or not the robot is sufficiently aligned to the specified
	 * target {@code Pose2d}.
	 * 
	 * @param targetPose the target {@code Pose2d}
	 * @return {@code true} if the robot is sufficiently aligned to the specified
	 *         target {@code Pose2d}; {@code false} otherwise
	 */
	boolean at(Pose2d targetPose) {
		var diff = targetPose.minus(m_driveSubsystem.getPose());
		return diff.getTranslation().getNorm() < m_controllerXY.getPositionTolerance()
				&& Math.abs(diff.getRotation().getRadians()) < m_controllerYaw.getPositionTolerance();
	}

	/**
	 * Returns the {@code IterativeTargetPoseSupplier} used by this
	 * {@code PathDriveCommand}.
	 * 
	 * @return the {@code IterativeTargetPoseSupplier} used by this
	 *         {@code PathDriveCommand}
	 */
	protected IterativeTargetPoseSupplier targetPoseSupplier() {
		return (IterativeTargetPoseSupplier) m_targetPoseSupplier;
	}

	/**
	 * An {@code IterativeTargetPoseSupplier} is a {@code Supplier<Pose2d>} that
	 * iterates over a number of {@code Supplier<Pose2d>}s.
	 * The {@link get()} method of an {@code IterativeTargetPoseSupplier} returns
	 * the {@code Pose2d} from the current {@code Supplier<Pose2d>} in iteration.
	 */
	static class IterativeTargetPoseSupplier implements Supplier<Pose2d> {

		/**
		 * The {@code Supplier<Pose2d>}s that provide the {@code Pose2d}s to which the
		 * robot should move.
		 */
		List<Supplier<Pose2d>> m_targetPoseSuppliers;

		/**
		 * The index indicating the current {@code Supplier<Pose2d>} in iteration.
		 */
		protected int m_targetPoseIndex = 0;

		/**
		 * Constructs an {@code IterativeTargetPoseSupplier}.
		 * 
		 * @param targetPoseSuppliers {@code Supplier<Pose2d>}s that provide the
		 *        {@code Pose2d}s to which the robot should move
		 */
		public IterativeTargetPoseSupplier(List<Supplier<Pose2d>> targetPoseSuppliers) {
			m_targetPoseSuppliers = targetPoseSuppliers;
		}

		/**
		 * Resets this {@code IterativeTargetPoseSupplier}.
		 */
		public void reset() {
			m_targetPoseIndex = 0;
		}

		/**
		 * Returns the curent target {@code Pose2d}.
		 * 
		 * @return the curent target {@code Pose2d}
		 */
		@Override
		public Pose2d get() {
			return m_targetPoseSuppliers.get(m_targetPoseIndex++).get();
		}

		/**
		 * Determines whether or not there are more {@code Supplier<Pose2d>}s to iterate
		 * over.
		 * 
		 * @return {@code true} if there are more {@code Supplier<Pose2d>}s to iterate
		 *         over; {@code false} otherwise
		 */
		public boolean hasNext() {
			return m_targetPoseIndex < m_targetPoseSuppliers.size();
		}
	}
}