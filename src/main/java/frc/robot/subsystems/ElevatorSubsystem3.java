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

	double m_ffVoltage = m_ff.calculate(0);
	Double m_position = null;

	@Override
	public Command goToLevel(DoubleSupplier level) {
		return startRun(() -> {
			m_setPosition = level.getAsDouble();
			m_position = null;
			SmartDashboard.putNumber("Elevator/Goal", m_setPosition);
		}, () -> {
			var p = getPosition();
			if (m_position != null)
				if (m_setPosition > p && p <= m_position + 1e-3)
					m_ffVoltage += 0.005;
				else
					m_ffVoltage *= 0.999;
			SmartDashboard.putNumber("Elevator/FFVoltage", m_ffVoltage);
			setPosition(m_setPosition, m_ffVoltage);
			m_position = p;
		}).until(() -> atSetpoint());
	}

}