// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem3 extends ElevatorSubsystem {
	public ElevatorSubsystem3(MechanismRoot2d root) {
		super(root);
	}

	public static double kFF = 1.56;

	@Override
	public Command goToLevel(DoubleSupplier level) {
		return startRun(() -> {
			setPosition(level.getAsDouble(), kFF);
			SmartDashboard.putNumber("Elevator/Goal", m_setPosition);
		}, () -> {
		}).until(() -> atSetpoint());
	}

}