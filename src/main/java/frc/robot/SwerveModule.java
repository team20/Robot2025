// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_steerController = new PIDController(kP, kI, kD);
	protected final CANcoder m_CANCoder;
	protected final TalonFX m_driveMotor;
	protected final SparkFlex m_steerMotor;

	public SwerveModule(int canId, int drivePort, int steerPort) {
		m_CANCoder = new CANcoder(canId);
		m_driveMotor = new TalonFX(drivePort);
		m_steerMotor = new SparkFlex(steerPort, MotorType.kBrushless);
		m_driveMotor.getConfigurator().apply(DriveConstants.kDriveConfig);
		var config = new SparkMaxConfig();
		// MK4i modules need their steer motors inverted
		config.idleMode(IdleMode.kBrake).voltageCompensation(12).inverted(true);
		config.openLoopRampRate(kRampRate).closedLoopRampRate(kRampRate);
		// Helps with encoder precision (not set in stone)
		config.encoder.uvwAverageDepth(kEncoderDepth).uvwMeasurementPeriod(kEncoderMeasurementPeriod);
		config.smartCurrentLimit(kSteerSmartCurrentLimit).secondaryCurrentLimit(kSteerSecondaryCurrentLimit);
		m_steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_steerController.enableContinuousInput(0, 360);
	}

	/**
	 * Returns the drive encoder distance in meters.
	 * 
	 * @return the drive encoder position in meters
	 */
	public double getDriveEncoderPosition() {
		return m_driveMotor.getPosition().getValueAsDouble() * kMetersPerMotorRotation;
	}

	/**
	 * Returns the current of the steer motor
	 * 
	 * @return The current in amps
	 */
	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	/**
	 * Returns the current of the drive motor
	 * 
	 * @return The current in amps
	 */
	public double getDriveCurrent() {
		return m_driveMotor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Resets the drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveMotor.setPosition(0);
	}

	/**
	 * Returns the current drive motor voltage.
	 * 
	 * @return the motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_driveMotor.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Returns the current drive motor temperature.
	 * 
	 * @return the temperature in degrees Celsius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getDeviceTemp().getValueAsDouble();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return the module angle in degrees
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Returns the current {@code SwerveModulePosition} of this
	 * {@code SwerveModule}.
	 * 
	 * @return the current {@code SwerveModulePosition} of this {@code SwerveModule}
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Returns the current {@code SwerveModuleState} of this {@code SwerveModule}.
	 * 
	 * @return the current {@code SwerveModuleState} of this {@code SwerveModule}
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle of this {@code SwerveModule}.
	 * 
	 * @param state a {@code SwerveModuleState} containing the target speeds and
	 *        angle
	 */
	public void setModuleState(SwerveModuleState state) {
		m_driveMotor.setVoltage(state.speedMetersPerSecond);
		double turnPower = m_steerController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_steerMotor.setVoltage(turnPower);
	}

}
