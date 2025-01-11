package frc.robot;

import frc.robot.subsystems.DriveSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;

	public CommandComposer(DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;
	}
}
