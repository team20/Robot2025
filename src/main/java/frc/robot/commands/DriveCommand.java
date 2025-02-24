package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code DriveCommand} aims to maneuver the robot to a certain
 * {@code Pose2d}. It utilizes two {@code ProfiledPIDController}s to precisely
 * control the robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand}.
	 */
	protected DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the x and y
	 * dimensions in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	protected ProfiledPIDController m_controllerXY;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the yaw
	 * dimension in radians (input: error in radians, output: velocity in radians
	 * per second).
	 */
	protected ProfiledPIDController m_controllerYaw;

	/**
	 * The {@code Pose2d} to which the robot should move.
	 */
	protected Pose2d m_targetPose;

	/**
	 * The {@code Supplier<Pose2d>} that provides the {@code Pose2d} to which the
	 * robot should move.
	 */
	protected Supplier<Pose2d> m_targetPoseSupplier;

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move
	 * the robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param targetPose a {@code Pose2d} to which the robot should move.
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, double distanceTolerance, double angleToleranceInDegrees,
			Pose2d targetPose) {
		this(driveSubsystem, distanceTolerance, angleToleranceInDegrees, () -> targetPose);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move
	 * the robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move. This is used at the
	 *        commencement of the {@code DriveCommand} (i.e., when the scheduler
	 *        begins to periodically execute the {@code DriveCommand})
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, double distanceTolerance, double angleToleranceInDegrees,
			Supplier<Pose2d> targetPoseSupplier) {
		m_driveSubsystem = driveSubsystem;
		m_controllerXY = new ProfiledPIDController(kDriveP, kDriveI, kDriveD,
				new TrapezoidProfile.Constraints(kDriveMaxSpeed, kDriveMaxAcceleration));
		m_controllerYaw = new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
				new TrapezoidProfile.Constraints(kTurnMaxAngularSpeed,
						kTurnMaxAcceleration));
		m_controllerXY.setTolerance(distanceTolerance);
		m_controllerYaw.setTolerance(Math.toRadians(angleToleranceInDegrees));
		m_controllerYaw.enableContinuousInput(0, 2 * Math.PI);
		m_targetPoseSupplier = targetPoseSupplier;
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveCommand}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_driveSubsystem.getPose();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
			e.printStackTrace();
			m_targetPose = pose;
		}
		m_controllerXY.reset(m_targetPose.minus(pose).getTranslation().getNorm());
		m_controllerYaw.reset(pose.getRotation().getRadians());
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		m_driveSubsystem.drive(chassisSpeeds(), true);
	}

	/**
	 * Calculates the {@code ChassisSpeeds} to move the robot toward the target.
	 * 
	 * @return the calculated {@code ChassisSpeeds} to move the robot toward the
	 *         target
	 */
	public ChassisSpeeds chassisSpeeds() {
		var currentPose = m_driveSubsystem.getPose();
		Translation2d current2target = m_targetPose.getTranslation()
				.minus(currentPose.getTranslation());
		double velocityX = 0, velocityY = 0;
		double distance = current2target.getNorm();
		double speed = Math.abs(m_controllerXY.calculate(distance, 0));
		if (speed > 1e-6) { // due to implementation of Translation2d#getAngle()
			var angle = current2target.getAngle();
			if (speed > kDriveMinSpeed) {
				velocityX = speed * angle.getCos();
				velocityY = speed * angle.getSin();
			} else {
				velocityX = kDriveMinSpeed * angle.getCos();
				velocityY = kDriveMinSpeed * angle.getSin();
			}
		}
		double angularVelocityRadiansPerSecond = m_controllerYaw
				.calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());
		if (Math.abs(angularVelocityRadiansPerSecond) < kTurnMinAngularSpeed)
			angularVelocityRadiansPerSecond = Math.signum(angularVelocityRadiansPerSecond) * kTurnMinAngularSpeed;
		return new ChassisSpeeds(velocityX, velocityY, angularVelocityRadiansPerSecond);
	}

	/**
	 * Is invoked once this {@code DriveCommand} is either ended or
	 * interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveCommand} was
	 *        interrupted
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
		return m_controllerXY.atGoal() && m_controllerYaw.atGoal();
	}

}