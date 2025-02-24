// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.WristConstants.*;

import java.util.function.DoubleSupplier;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SimpleWristSubsystem extends SubsystemBase {
	protected final SparkMax m_wristMotor = new SparkMax(kWristMotorPort, MotorType.kBrushless);
	private final SparkAbsoluteEncoder m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder();
	private final SparkClosedLoopController m_wristClosedLoopController = m_wristMotor.getClosedLoopController();

	private double m_targetAngle = 0;
	// Adjust ramp rate, step voltage, and timeout to make sure wrist doesn't break
	private final SysIdRoutine m_sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(Volts.of(2.5).div(Seconds.of(1)), Volts.of(3), Seconds.of(3)),
			new SysIdRoutine.Mechanism(m_wristMotor::setVoltage, null, this));

	/** Creates a new WristSubsystem. */
	public SimpleWristSubsystem() {
		var config = new SparkMaxConfig();
		config
				.inverted(false)
				.idleMode(IdleMode.kBrake) // TODO: Soft limits?
				.smartCurrentLimit(kSmartCurrentLimit)
				.secondaryCurrentLimit(kSecondaryCurrentLimit)
				.voltageCompensation(12);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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
	 * @return the angle of the wrist (degrees)
	 */
	public double getAngle() {
		return m_absoluteEncoder.getPosition() * 360;
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
	 * Determine the output current of the motor
	 * 
	 * @return The max value that the output current gets to
	 */
	public double getOutputCurrent() {
		return Math.abs(m_wristMotor.getOutputCurrent());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Output current", getOutputCurrent());
		SmartDashboard.putNumber("Angle", getAngle());
		SmartDashboard.putNumber("Applied output", m_wristMotor.getAppliedOutput());
	}

	/**
	 * Stops the motor
	 * 
	 * @return Sets the speed to 0
	 */
	public Command stopMotor() {
		return runOnce(() -> setSpeed(0)).withName("Stop Wrist motor");
	}

	/**
	 * Reverse the motor by setting the speeed to negative
	 * 
	 * @return the command to set the speed
	 */
	public Command reverseMotor() {
		return runOnce(() -> setSpeed(-1)).withName("Reverse Wrist motor");
	}

	/**
	 * Make the motor spin forward with a positive speed
	 * 
	 * @return the command to set the speed
	 */
	public Command forwardMotor() {
		return runOnce(() -> setSpeed(1)).withName("Wrist motor forward");
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
				m_wristMotor.set(speed * 0.5);
			} else {
				m_wristMotor.stopMotor();
			}

		}).withName("Manual Wrist");
	}

	/**
	 * Sets the angle of the motors
	 * 
	 * @param position angle (in degrees) to set the wrist to
	 */
	public Command goToAngle(double angle) {
		return runOnce(() -> {
			m_targetAngle = angle / 360;
			m_wristClosedLoopController.setReference(m_targetAngle, ControlType.kPosition);
		}).withName("Wrist go to angle");
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidDynamic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.dynamic(direction);
	}

	/**
	 * Creates a {@code Command} for testing this {@code SimpleWristSubsystem}
	 * (Levels 4 and others).
	 * 
	 * @param duration the duration of each movement in seconds
	 * 
	 * @return a {@code Command} for testing this {@code SimpleWristSubsystem}
	 */
	public Command testCommand(double duration) {
		return sequence(
				goToAngle(kGrabberAngleLevelFour), new WaitCommand(duration), goToAngle(kGrabberAngleOthers),
				new WaitCommand(duration));
	}

}