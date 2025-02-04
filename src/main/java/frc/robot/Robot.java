// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.AlgaeGrabberSubsystem.GrabberState;
import frc.robot.subsystems.CheeseStickSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private final AlgaeGrabberSubsystem m_algaeGrabberSubsystem = new AlgaeGrabberSubsystem();
	private final CheeseStickSubsystem m_cheeseStickSubsystem = new CheeseStickSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
	private final WristSubsystem m_wristSubsystem = new WristSubsystem();
	// Changed to PS5
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(kOperatorControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();

	public Robot() {
		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(
				Map.of(
						11, "FR Turn", 21, "BR Turn", 31, "BL Turn", 41, "FL Turn", kElevatorMotorPort, "Elevator",
						kClimberMotorPort, "Climber Motor", kWristMotorPort, "Wrist Motor", kFlywheelMotorPort,
						"Algae Motor"));
		DriverStation.startDataLog(DataLogManager.getLog());
		bindDriveControls();
		bindWristControls();
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getSquareButton));
		// TODO: Add in joystick rotation
	}

	public void bindElevatorControls() {
		m_operatorController.circle().onTrue(m_elevatorSubsystem.goToLevelFourCommand());
		m_operatorController.triangle().onTrue(m_elevatorSubsystem.goToLevelThreeCommand());
		m_operatorController.square().onTrue(m_elevatorSubsystem.goToLevelTwoCommand());
		m_operatorController.cross().onTrue(m_elevatorSubsystem.goToLevelOneCommand());
		m_operatorController.povLeft().onTrue(m_elevatorSubsystem.goToCoralStationCommand());
		// TODO: Add manual movement
	}

	public void bindAlgaeControls() {
		m_operatorController.R1().onTrue(
				m_algaeGrabberSubsystem.deployGrabberCommand(GrabberState.DOWN)
						.andThen(m_algaeGrabberSubsystem.runFlywheelCommand())); // TODO: Come up after?
		m_operatorController.L1().onTrue(
				m_algaeGrabberSubsystem.runFlywheelReverseCommand()
						.until(m_algaeGrabberSubsystem::checkCurrentOnFlywheel)
						.andThen(m_algaeGrabberSubsystem.stopFlywheelCommand()));
	}

	public void bindWristControls() {
		m_driverController.circle().onTrue(m_wristSubsystem.reverseMotor());
		m_driverController.square().onTrue(m_wristSubsystem.forwardMotor());
	}

	public void bindCheeseStickControls() {
		// Need to figure out what left and right actually mean
		m_operatorController.R2().whileFalse(m_cheeseStickSubsystem.goLeft());
		m_operatorController.R2().whileTrue(m_cheeseStickSubsystem.goRight());
	}

	public void bindClimberControls() {
		m_operatorController.L2().whileTrue(m_climberSubsystem.moveForward())
				.onFalse(m_climberSubsystem.moveBackward());
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
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
