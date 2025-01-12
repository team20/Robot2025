// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Notes:
 * Two motors, encoders, how it turns
 * Gear Ratio: 
 * Normal Rev motor
 * 	Maybe Kraken
 */

//TODO: Setpoint and hold?

public class ElevatorSubsystem extends SubsystemBase {

	private final SparkMax m_masterMotor = new SparkMax(kElevatorMotorRightPort, MotorType.kBrushless);
	private final SparkMax m_followerMotor = new SparkMax(kElevatorMotorLeftPort, MotorType.kBrushless);

	private final RelativeEncoder m_masterEncoder = m_masterMotor.getEncoder(); // TODO: Is this the right encoder?

	private final SparkClosedLoopController m_closedLoopController = m_masterMotor.getClosedLoopController();

	private double m_setPositionLeft = 0;

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {
		var config = new SparkMaxConfig();
		config
				.inverted(false)
				.idleMode(IdleMode.kBrake) // TODO: Soft limits?
				.smartCurrentLimit(kSmartCurrentLimit)
				.secondaryCurrentLimit(kSecondaryCurrentLimit)
				.voltageCompensation(12);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(kP, kI, kD);

		var followerConfig = new SparkMaxConfig();
		followerConfig.apply(config);
		followerConfig
				.follow(kElevatorMotorRightPort); // TODO: INVERTED?

		m_masterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		resetEncoder();
	}

	/**
	 * This method sets the speed of each of the motors
	 * 
	 * @param speed the speed you want to set it to
	 */
	public void setSpeed(double speed) {
		m_masterMotor.set(speed);
	}

	/**
	 * Gets the position of the left motor
	 * 
	 * @return the position of the left motor (through the encoder value)
	 */
	public double getleftPosition() {
		return m_masterEncoder.getPosition();
	}

	/**
	 * Tests if within the tolerance of the setpoint of the left motor
	 * 
	 * @return true if at the setpoint
	 */
	public boolean atleftSetpoint() {
		return (Math.abs(m_setPositionLeft - getleftPosition()) <= kTolerance);
	}

	/**
	 * Sets the position of the motors (using PID)
	 * 
	 * @param position Position to set the master motor to (the follower
	 *        will...follow)
	 */
	public void setPosition(double position) {
		m_setPositionLeft = position;
		m_closedLoopController.setReference(position, ControlType.kPosition);
	}

	/**
	 * Resets the encoder value to 0 (starting position)
	 */
	public void resetEncoder() {
		m_masterEncoder.setPosition(0);
		setPosition(0);
	}

	/**
	 * Determine the output current of the motors
	 * 
	 * @return The max value that the output current gets to
	 */
	public double getOutputCurrent() {
		return Math.abs(m_masterMotor.getOutputCurrent()); // TODO: Needs other motor?
	}

	/**
	 * Checks if position is at 0 (and within tolerance)
	 * 
	 * @return true if within tolerance
	 */
	public boolean atBase() {
		return (Math.abs(getleftPosition()) <= kTolerance);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/**
	 * Stops the motor
	 * 
	 * @return Sets the speed to 0
	 */
	public Command stopMotor() {
		return runOnce(() -> setSpeed(0));
	}

	/**
	 * Reverse the motor by setting the speeed to negative
	 * 
	 * @return the command to set the speed
	 */
	public Command reverseMotor() {
		return runOnce(() -> setSpeed(-1));
	}

	/**
	 * Make the motor spin forward with a positive speed
	 * 
	 * @return the command to set the speed
	 */
	public Command forwardMotor() {
		return runOnce(() -> setSpeed(1));
	}

	public Command setPositionBase() {
		return runOnce(() -> setPosition(0));
	}

	public Command setPositionMiddle() {
		return runOnce(() -> setPosition(kMaxExtension / 2));
	}

	public Command setPositionTop() {
		return runOnce(() -> setPosition(kMaxExtension));
	}

}
