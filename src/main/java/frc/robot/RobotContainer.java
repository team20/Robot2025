// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
	private DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
	private SendableChooser<Command> m_autoSelector;

	public RobotContainer() {
		configureBindings();

		m_autoSelector.addOption("Leave", m_DriveSubsystem.driveForTimeCommand(2));
		SmartDashboard.putData(m_autoSelector);
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
