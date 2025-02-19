// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.AlgaeConstants;

public class AlgaeGrabberSubsystem extends SubsystemBase {
	private final SparkMax m_flywheel;
	private final SparkMax m_grabberAngleMotor = new SparkMax(AlgaeConstants.kGrabberAnglePort,
			MotorType.kBrushless);
	private final SparkClosedLoopController m_grabberClosedLoopController = m_grabberAngleMotor
			.getClosedLoopController();
	private final Debouncer m_debouncerCurrentLimitStop;

	private double m_setVelocity;

	public enum GrabberState {
		UP,
		DOWN
	}

	public AlgaeGrabberSubsystem() {
		m_debouncerCurrentLimitStop = new Debouncer(AlgaeConstants.kTimeOverCurrentToStop);
		m_flywheel = new SparkMax(AlgaeConstants.kFlywheelMotorPort, MotorType.kBrushless);

		// Initialize Motors
		// applies the inverts and coast mode to the flywheel motors. if you want to
		// make a motor spin the other way change it in the AlgaeConstants.k____Invert
		// variable
		// also applies voltage and current stuff to the motors

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(AlgaeConstants.kFlywheelInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(AlgaeConstants.k550SmartCurrentLimit);
		m_flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		config = new SparkMaxConfig(); // To use the same variable in order to reset the parameters for the other motor
		config.inverted(AlgaeConstants.kGrabberAngleInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(AlgaeConstants.kSmartCurrentLimit);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.pid(AlgaeConstants.kP, AlgaeConstants.kI, AlgaeConstants.kD);
		m_grabberAngleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
	}

	/**
	 * gets the % velocity on the flywheel motor
	 *
	 * @return (double) (-1 to 1) returns the % velocity using .getAppliedOutput()
	 *         from {@link SparkMax}
	 */
	public double getVelocity() {
		return m_flywheel.getAppliedOutput();
	}

	/**
	 * sets the speed for the wheels to grab the Algae balls
	 *
	 * @param velocity (double) (-1 to 1) sets m_setVelocity and changes the wheel
	 *        to go that speed using .set from {@link SparkMax}
	 * @return (void) motor will spin velocity
	 */
	public void setVelocity(double velocity) {
		m_setVelocity = velocity;
		m_flywheel.set(m_setVelocity);
	}

	/**
	 * checks the current draw on the flywheel motor and will check if it's lower or
	 * higher than AlgaeConstants.kCurrentToStop using a debouncer
	 *
	 * @return (boolean) true -> if the current draw is higher or equal than
	 *         kCurrentToStop
	 *         <p>
	 *         (boolean) false -> otherwise
	 */
	public boolean checkCurrentOnFlywheel() {
		return m_debouncerCurrentLimitStop
				.calculate(m_flywheel.getOutputCurrent() >= AlgaeConstants.kCurrentToStop);
	}

	/**
	 * command to stop the algae flywheel using the setVelocity method
	 *
	 * @return (setVelocity(0)) sets the flywheel velocity to 0%
	 */
	public Command stopFlywheel() {
		return runOnce(() -> {
			setVelocity(0);
		});
	}

	/**
	 * command to slow run the algae flywheel using the setVelocity method
	 *
	 * @return (setVelocity(0.01)) sets the flywheel velocity to 1%
	 */
	public Command slowRunFlywheel() {
		return runOnce(() -> {
			setVelocity(0.01);
		});
	}

	/**
	 * command to start the algae flywheel using the setVelocity method
	 *
	 * @return (setVelocity(kFlywheelSpeed)) sets the flywheel velocity to the
	 *         constant kFlywheelSpeed in AlgaeConstants
	 */
	public Command runFlywheel() {
		return run(() -> {
			setVelocity(AlgaeConstants.kFlywheelSpeed);
		});
	}

	/**
	 * command to reverse the algae flywheel using the setVelocity method
	 *
	 * @return (setVelocity(-kFlywheelSpeed)) sets the flywheel velocity to the
	 *         negative constant kFlywheelSpeed in AlgaeConstants
	 */
	public Command runFlywheelReverse() {
		return runOnce(() -> {
			setVelocity(-AlgaeConstants.kFlywheelSpeed);
		});
	}

	/**
	 * deploys the grabber for algae
	 *
	 * @param state (enum grabberState) can be UP or DOWN, UP will bring the grabber
	 *        up, DOWN will bring it down
	 * 
	 * @return moves the whole grabber setup using a PID based on the grabberState
	 *         enum given
	 */
	public Command deployGrabber(GrabberState state) {
		return runOnce(() -> {
			if (GrabberState.DOWN == state) {
				m_grabberClosedLoopController
						.setReference(AlgaeConstants.kDeployGrabberRotations, ControlType.kPosition);
			} else if (GrabberState.UP == state) {
				m_grabberClosedLoopController.setReference(0, ControlType.kPosition);
			}
		});
	}
}
