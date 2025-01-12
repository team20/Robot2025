// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
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

public class WristSubsystem extends SubsystemBase {
	private final SparkMax m_wristMotor = new SparkMax(kWristPort, MotorType.kBrushless);

	private final SparkAbsoluteEncoder m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder();

	private final SparkClosedLoopController m_wristClosedLoopController = m_wristMotor.getClosedLoopController();

	private double m_targetAngle = 0;

	/** Creates a new WristSubsystem. */
	public WristSubsystem() {
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
		m_wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	/**
	 * This method sets the speed
	 * 
	 * @param speed the speed you want to set it to
	 */
	public void setSpeed(double speed) {
		m_wristMotor.set(speed);
	}

	/**
	 * @return the angle of the wrist
	 */
	public double getAngle() {
		return m_absoluteEncoder.getPosition();
	}

	/**
	 * Tests if within the tolerance of the angle of the motor
	 * 
	 * @return true if at the setpoint
	 */
	public boolean atAngle() {
		return (Math.abs(m_targetAngle - getAngle()) <= kTolerance);
	}

	/**
	 * Sets the angle of the motors (using PID)
	 * 
	 * @param position angle to set the wrist to
	 */
	public void setAngle(double position) {
		m_targetAngle = position;
		m_wristClosedLoopController.setReference(position, ControlType.kPosition);
	}

	/**
	 * Determine the output current of the motor
	 * 
	 * @return The max value that the output current gets to
	 */
	public double getOutputCurrent() {
		return Math.abs(m_wristMotor.getOutputCurrent());
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

}
