// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SimpleElevatorSubsystem extends ElevatorSubsystem {
	public SimpleElevatorSubsystem(MechanismRoot2d root) {
		super(root);
	}

	/**
	 * Using Trapezoid Profile to set the position of the elevator
	 * 
	 * @param level A function that returns the level we want to go to
	 * @return the command
	 */
	@Override
	public Command goToLevel(DoubleSupplier level) {
		return startRun(() -> {
			setPosition(level.getAsDouble(), 1);
			SmartDashboard.putNumber("Elevator/Goal", m_setPosition);
		}, () -> {
		}).until(() -> atSetpoint());
	}
}