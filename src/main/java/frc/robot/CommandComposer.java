package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static PoseEstimationSubsystem m_poseEstimationSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_poseEstimationSubsystem = poseEstimationSubsystem;
	}

	/**
	 * Returns a {@code Command} for testing the rotation capability of the robot.
	 * 
	 * @return a {@code Command} for testing the rotation capability of the robot
	 */
	public static Command testRotation() {
		double rotionalSpeed = kTurnMaxAngularSpeed * 0.9;
		double duration = 2.0;
		return sequence(
				m_driveSubsystem.run(() -> m_driveSubsystem.setModuleAngles(0)).withTimeout(1),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(.5, 0, rotionalSpeed, true))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(-.5, 0, -rotionalSpeed, true))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, rotionalSpeed, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.setModuleAngles(0)).withTimeout(1),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, -rotionalSpeed, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.setModuleAngles(0)).withTimeout(1),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, rotionalSpeed, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.setModuleAngles(0)).withTimeout(1),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, -rotionalSpeed, false))
						.withTimeout(duration));
	}

	/**
	 * Returns a {@code Command} for turning the robot toward the specified
	 * {@code AprilTag}.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @return a {@code Command} for turning the robot toward the specified
	 *         {@code AprilTag}
	 */
	public static Command turnTowardTag(int tagID) {
		return m_driveSubsystem.run(() -> {
			Rotation2d a = m_poseEstimationSubsystem.angularDisplacement(1);
			if (a != null)
				m_driveSubsystem.drive(
						0, 0, a.getDegrees() * kTurnP,
						false);
		}).withTimeout(1);
	}

}
