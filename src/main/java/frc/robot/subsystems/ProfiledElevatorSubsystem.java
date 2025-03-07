// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ProfiledElevatorSubsystem extends ElevatorSubsystem {
	private final Timer m_timer = new Timer();
	private final ElevatorFeedforward m_ff = new ElevatorFeedforward(kS, kG, kV, kA);
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccel));
	// Adjust ramp rate, step voltage, and timeout to ensure elevator doesn't break

	/** Creates a new ElevatorSubsystem. */
	public ProfiledElevatorSubsystem(MechanismRoot2d root) {
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
		var initial = new TrapezoidProfile.State();
		var finalState = new TrapezoidProfile.State();

		return startRun(() -> {
			m_timer.restart();
			initial.position = getPosition();
			finalState.position = level.getAsDouble();
			SmartDashboard.putNumber("Elevator/Goal", level.getAsDouble());
		}, () -> {
			double time = m_timer.get();
			TrapezoidProfile.State currentState = m_profile.calculate(time, initial, finalState);
			TrapezoidProfile.State nextState = m_profile.calculate(time + 0.02, initial, finalState);
			double ff = m_ff.calculateWithVelocities(currentState.velocity, nextState.velocity);
			setPosition(nextState.position, ff);
			SmartDashboard.putNumber("Elevator/Current Target Position", currentState.position);
			SmartDashboard.putNumber("Elevator/Current Target Velocity", currentState.velocity);
			SmartDashboard.putNumber("Elevator/Next Target Position", nextState.position);
			SmartDashboard.putNumber("Elevator/Next Target Velocity", nextState.velocity);
			SmartDashboard.putNumber("Elevator/Profile Time", m_profile.totalTime());
			SmartDashboard.putNumber("Elevator/Current Time", m_timer.get());
		}).until(() -> Math.abs(finalState.position - getPosition()) <= kTolerance);
		// }).until(() -> m_profile.isFinished(m_timer.get()));
	}

}