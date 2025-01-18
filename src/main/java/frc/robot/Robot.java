// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final CommandPS4Controller m_driverController = new CommandPS4Controller(
			ControllerConstants.kDriverControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();

	public Robot() {
		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(Map.of(11, "FR Turn", 21, "BR Turn", 31, "BL Turn", 41, "FL Turn"));
		DriverStation.startDataLog(DataLogManager.getLog());
		bindDriveControls();
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						() -> m_driverController.getR2Axis() - m_driverController.getL2Axis(),
						m_driverController.getHID()::getSquareButton));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = null;

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		// we can test critical subsystems and commands here.
		m_driveSubsystem.testCommand() // test DriveSubsystem#drive(...)
				// move forward, backward, strafe left, strafe right, and turn left and right
				.andThen(new DriveCommand(m_driveSubsystem, new Pose2d(.3, .3, Rotation2d.fromDegrees(90)), .1, 3))
				// test DriveCommand: move to (.2, .2, 90)
				.andThen(new DriveCommand(m_driveSubsystem, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), .1, 3))
				// test DriveCommand: move to (0, 0, 0)
				.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
