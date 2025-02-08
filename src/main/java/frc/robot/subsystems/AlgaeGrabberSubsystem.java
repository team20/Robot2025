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

	private final Debouncer m_debouncerCurrentLimitStop = new Debouncer(AlgaeConstants.kTimeOverCurrentToStop);

	private double m_setVelocity;

	public enum GrabberState {
		UP,
		DOWN
	}

	public AlgaeGrabberSubsystem() {
		m_flywheel = new SparkMax(AlgaeConstants.kFlywheelMotorPort, MotorType.kBrushless);

		// Initialize Motors
		// applies the inverts and coast mode to the flywheel motors. if you want to
		// make a motor spin the other way change it in the AlgaeConstants.k____Invert
		// variable
		// also applies voltage and current stuff to the motors

		var config = new SparkMaxConfig();
		config.inverted(AlgaeConstants.kFlywheelInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(AlgaeConstants.k550SmartCurrentLimit);
		m_flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		config = new SparkMaxConfig();
		config.inverted(AlgaeConstants.kGrabberAngleInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(AlgaeConstants.kSmartCurrentLimit);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(AlgaeConstants.kP, AlgaeConstants.kI, AlgaeConstants.kD);
		m_grabberAngleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// use this to check what the kCurrentToStop should be
		// System.out.println(m_flywheel.getOutputCurrent());
	}

	public double getVelocity() {
		return m_flywheel.getAbsoluteEncoder().getVelocity();
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
	 * @return (setVelocity(0)) sets the flywheel velocity to 0
	 */
	public Command stopFlywheelCommand() {
		return runOnce(() -> {
			setVelocity(0);
		});
	}

	/**
	 * command to start the algae flywheel using the setVelocity method
	 *
	 * @return (setVelocity(.75)) sets the flywheel velocity to 75%
	 */
	public Command runFlywheelCommand() {
		return runOnce(() -> {
			setVelocity(.75);
		});
	}

	/**
	 * command to reverse the algae flywheel using the setVelocity method
	 *
	 * @return (setVelocity(-.75)) sets the flywheel velocity to -75%
	 */
	public Command runFlywheelReverseCommand() {
		return runOnce(() -> {
			setVelocity(-.75);
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
	public Command deployGrabberCommand(GrabberState state) {
		return runOnce(() -> {
			if (GrabberState.DOWN == state) {
				m_grabberClosedLoopController.setReference(2, ControlType.kPosition);
			} else if (GrabberState.UP == state) {
				m_grabberClosedLoopController.setReference(0, ControlType.kPosition);
			}
		});
	}
}