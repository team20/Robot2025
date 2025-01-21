package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code DriveCommand} aims to maneuver the robot from its current pose to
 * a certain target pose.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier} providing the current {@code Pose2d} of the robot.
	 */
	private Supplier<Pose2d> m_poseSupplier;

	/**
	 * The {@code Supplier<Pose2d>} that calculates the target pose to which the
	 * robot should move.
	 * This is used at the commencement of this {@code DriveCommand} (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveCommand}).
	 */
	private Supplier<Pose2d> m_targetPoseSupplier;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the x
	 * dimension in meters.
	 */
	private ProfiledPIDController m_controllerX;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the y
	 * dimension in meters.
	 */
	private ProfiledPIDController m_controllerY;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the yaw
	 * dimension in angles.
	 */
	private ProfiledPIDController m_controllerYaw;

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPose the target pose to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier, Pose2d targetPose,
			double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, poseSupplier, () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        target pose to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_poseSupplier = poseSupplier;
		m_targetPoseSupplier = targetPoseSupplier;
		var constraints = new TrapezoidProfile.Constraints(kDriveMaxVelocity, kDriveMaxAcceleration);
		m_controllerX = new ProfiledPIDController(kDriveP, kDriveI, kDriveD, constraints);
		m_controllerY = new ProfiledPIDController(kDriveP, kDriveI, kDriveD, constraints);
		m_controllerYaw = new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
				new TrapezoidProfile.Constraints(kTurnMaxVelocity, kTurnMaxAcceleration));
		m_controllerX.setTolerance(distanceTolerance);
		m_controllerY.setTolerance(distanceTolerance);
		m_controllerYaw.setTolerance(angleTolerance);
		m_controllerYaw.enableContinuousInput(-180, 180);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Constructs a {@code Commmand} for moving the robot toward the specified
	 * target position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPositionSupplier a {@code Supplier<Pose2d>} that provides the
	 *        target position.
	 *        This is used at the commencement of the
	 *        {@code DriveCommand} (i.e., when the scheduler
	 *        begins to periodically execute the
	 *        {@code DriveCommand})
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param distanceToTarget the desired distance between the robot and the
	 *        target position
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static Command moveToward(DriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier,
			Supplier<Pose2d> poseSupplier,
			double distanceToTarget,
			double distanceTolerance,
			double angleTolerance) {
		return new DriveCommand(driveSubsystem, poseSupplier,
				() -> poseSupplier.get()
						.plus(transformationToward(targetPositionSupplier.get(), poseSupplier.get(), distanceToTarget)),
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
	 * Is invoked at the commencement of this {@code DriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this {@code DriveCommand}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_poseSupplier.get();
		var targetPose = pose;
		try {
			targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
		}
		m_controllerX.reset(pose.getX());
		m_controllerY.reset(pose.getY());
		m_controllerYaw.reset(pose.getRotation().getDegrees());
		m_controllerX.setGoal(targetPose.getX());
		m_controllerY.setGoal(targetPose.getY());
		m_controllerYaw.setGoal(targetPose.getRotation().getDegrees());
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		Pose2d pose = m_poseSupplier.get();
		double speedX = m_controllerX.calculate(pose.getX());
		double speedY = m_controllerY.calculate(pose.getY());
		// NEGATION needed if the robot rotates clockwise given positive turnSpeed
		double speedYaw = m_controllerYaw.calculate(pose.getRotation().getDegrees());
		// speedX = applyThreshold(speedX, DriveConstants.kMinSpeed);
		// speedY = applyThreshold(speedY, DriveConstants.kMinSpeed);
		m_driveSubsystem.drive(speedX, speedY, speedYaw, true);
	}

	/**
	 * Is invoked once this {@code DriveCommand} is either ended or interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.drive(0, 0, 0, true);
	}

	/**
	 * Determines whether or not this {@code DriveCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerYaw.atGoal();
	}

	/**
	 * Applies the specified threshold to the specified value.
	 * 
	 * @param value the value to be thresholded
	 * @param threshold the threshold limit
	 * @return the original value if the absolute value of that value is greater or
	 *         equal to the threshold; the threshold with the original value's sign
	 *         otherwise
	 */
	public static double applyThreshold(double value, double threshold) {
		return Math.abs(value) < threshold ? Math.signum(value) * threshold : value;
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