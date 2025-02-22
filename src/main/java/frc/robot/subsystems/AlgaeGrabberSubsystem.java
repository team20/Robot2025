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

	private double m_setVelocity;

	public enum GrabberState {

		UP,
		DOWN
	}

	public AlgaeGrabberSubsystem() {
		// Initialize Motors
		// applies the inverts and coast mode to the flywheel motors. if you want to
		// make a motor spin the other way change it in the AlgaeConstants.k____Invert
		// variable
		// also applies voltage and current stuff to the motors

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(kFlywheelInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(kSmartCurrentLimit);
		m_flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		config = new SparkMaxConfig(); // To use the same variable in order to reset the parameters for the other motor
		config.inverted(kGrabberAngleInvert).idleMode(IdleMode.kBrake);
		config.voltageCompensation(12).smartCurrentLimit(kSmartCurrentLimit);
		config.voltageCompensation(12).secondaryCurrentLimit(kSecondaryCurrentLimit);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.pid(kP, kI, kD);
		m_grabberAngleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
			setVelocity(kFlywheelSpeed);
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
			setVelocity(-kFlywheelSpeed);
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
						.setReference(kDeployGrabberRotations, ControlType.kPosition);
			} else if (GrabberState.UP == state) {
				m_grabberClosedLoopController.setReference(0, ControlType.kPosition);
			}
		});
	}
}
