// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem2 extends ElevatorSubsystem {
	public ElevatorSubsystem2(MechanismRoot2d root) {
		super(root);
	}

	double voltage = 0.1;
	Double average = null;
	Double previousPosition = null;

	@Override
	public Command goToLevel(DoubleSupplier level) {
		return startRun(() -> {
			m_setPosition = level.getAsDouble();
		}, () -> {
			m_elevatorMotor.setVoltage(voltage);
			var p = getPosition();
			if (previousPosition != null)
				if (p < m_setPosition && p <= previousPosition)
					voltage *= 1.03;
			if (p > m_setPosition && (p >= previousPosition))
				voltage *= 0.97;
			average = average == null ? voltage : average * 0.99 + voltage * 0.01;
			if (average != null)
				SmartDashboard.putNumber("Elevator/Voltage", average);
			previousPosition = getPosition();
		});
	}
}