// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrabberSubsystem extends SubsystemBase {
	private final SparkMax m_flywheel = new SparkMax(kFlywheelMotorPort, MotorType.kBrushless);
	private final SparkMax m_grabberAngleMotor = new SparkMax(kGrabberAnglePort,
			MotorType.kBrushless);
	private final SparkClosedLoopController m_grabberClosedLoopController = m_grabberAngleMotor
			.getClosedLoopController();
	private final Debouncer m_debouncerCurrentLimitStop = new Debouncer(kTimeOverCurrentToStop);

	public AlgaeGrabberSubsystem() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(kFlywheelInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(kSmartCurrentLimit);
		m_flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		config = new SparkMaxConfig(); // To use the same variable in order to reset the parameters for the other motor
		config.inverted(kGrabberAngleInvert).idleMode(IdleMode.kBrake).voltageCompensation(12)
				.smartCurrentLimit(kSmartCurrentLimit)
				.secondaryCurrentLimit(kSecondaryCurrentLimit);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.pid(kP, kI, kD);
		m_grabberAngleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		// Causes Spark Maxes to send this data, allowing URCL to log it
		m_grabberAngleMotor.getEncoder().getPosition();
		m_grabberAngleMotor.getAbsoluteEncoder().getPosition();
	}

	@Override
	public void periodic() {
	}

	/**
	 * Allows the operator to manually move the Wrist for adjustment
	 * 
	 * @param joystick Input from operator's right joystick Y-values
	 * @return Command for moving
	 */
	public Command manualMove(DoubleSupplier joystick) {
		return run(() -> {
			double input = joystick.getAsDouble();
			double speed = Math.signum(input) * Math.pow(input, 2);
			if (Math.abs(speed) > 0.1) {
				m_grabberAngleMotor.set(speed * 0.5);
			} else {
				m_grabberAngleMotor.stopMotor();
			}

		}).withName("Manual Algae Grabber");
	}

	/**
	 * checks the current draw on the flywheel motor and will check if it's lower or
	 * higher than kCurrentToStop using a debouncer
	 *
	 * @return (boolean) true -> if the current draw is higher or equal than
	 *         kCurrentToStop
	 *         <p>
	 *         (boolean) false -> otherwise
	 */
	public boolean checkCurrentOnFlywheel() {
		return m_debouncerCurrentLimitStop
				.calculate(m_flywheel.getOutputCurrent() >= kSmartCurrentLimit);
	}

	/**
	 * Creates a command to grab algae.
	 * 
	 * @return The command.
	 */
	public Command grabAlgaeAndHold() {
		return run(() -> {
			m_grabberClosedLoopController.setReference(kDeployGrabberRotations, ControlType.kPosition);
			m_flywheel.set(kFlywheelSpeed);
		}).until(this::checkCurrentOnFlywheel).finallyDo(() -> {
			m_flywheel.set(0.01);
			m_grabberAngleMotor.set(0);
		});
	}

	/**
	 * Creates a command to reverse the flywheel direction and stop when the command
	 * ends.
	 * 
	 * @return The command.
	 */
	public Command reverseFlywheelAndStop() {
		return run(() -> m_flywheel.set(-kFlywheelSpeed)).finallyDo(() -> m_flywheel.set(0));
	}

	/**
	 * Creates a command to release algae.
	 * 
	 * @return The command.
	 */
	public Command releaseAlgae() {
		return run(() -> {
			m_grabberClosedLoopController.setReference(0, ControlType.kPosition);
			m_flywheel.set(-kFlywheelSpeed);
		}).withTimeout(1).finallyDo(() -> {
			m_flywheel.set(0);
			m_grabberAngleMotor.set(0);
		});
	}
}
