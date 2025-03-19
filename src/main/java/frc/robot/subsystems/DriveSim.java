package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DriveSim {
	private final SwerveDriveOdometry m_odometry;

	public DriveSim(SwerveDriveKinematics kinematics, Rotation2d heading, SwerveModulePosition[] modulePositions) {
		m_odometry = new SwerveDriveOdometry(kinematics, heading, modulePositions);
	}

	public void updateSimPose(Rotation2d heading, SwerveModulePosition[] modulePositions) {
		m_odometry.update(heading, modulePositions);
	}

	public void resetSimPose(Rotation2d heading, SwerveModulePosition[] modulePositions, Pose2d pose) {
		m_odometry.resetPosition(heading, modulePositions, pose);
	}

	public Pose2d getSimPose() {
		return m_odometry.getPoseMeters();
	}
}