// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Refer to the proper documentation here:
 * https://docs.google.com/presentation/d/1wI5ZK-UHxkvvEQYXKcCA2--MrBwhGgUsliOcLzugH3A/edit#slide=id.g2d4514aa178_1_6 
 * 
 * 
 */

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_PIDController = new PIDController(kP, kI, kD);
	private final CANcoder m_CANCoder;
	private final SparkMax m_driveMotor;
	private final RelativeEncoder m_driveEncoder;
	private final SparkMax m_steerMotor;
	private final DCMotorSim m_driveMotorModel = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(Units.radiansToRotations(0.12), Units.radiansToRotations(0.07)),
			DCMotor.getNEO(1), 0.0005);
	private final DCMotorSim m_steerMotorModel = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(Units.radiansToRotations(0.12), Units.radiansToRotations(0.07)),
			DCMotor.getNEO(1), 0.0005);

	private final SparkMax m_sparkMaxFrontRight = new SparkMax(
			Constants.DriveConstants.kFrontRightDrivePort,
			MotorType.kBrushless);
	private final SparkMax m_sparkMaxFrontLeft = new SparkMax(Constants.DriveConstants.kFrontLeftDrivePort,
			MotorType.kBrushless);
	private final SparkMax m_sparkMaxBackRight = new SparkMax(Constants.DriveConstants.kBackRightDrivePort,
			MotorType.kBrushless);
	private final SparkMax m_sparkMaxBackLeft = new SparkMax(Constants.DriveConstants.kBackLeftDrivePort,
			MotorType.kBrushless);

	public SwerveModule(int CANport, int drivePort, int steerPort) {
		m_CANCoder = new CANcoder(CANport); // Making a CANCoder Object
		m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);
		m_PIDController.setIZone(kIz);
		m_driveEncoder = m_driveMotor.getEncoder();
		configMotorController(m_driveMotor, kDriveSmartCurrentLimit, kDrivePeakCurrentLimit);
		configMotorController(m_steerMotor, kSteerSmartCurrentLimit, kSteerPeakCurrentLimit);
		m_PIDController.enableContinuousInput(0, 360);
	}

	/**
	 * Configures our motors with the exact same settings
	 * 
	 * @param motorController The CANSparkMax to configure
	 */
	private void configMotorController(SparkMax motorController, int smartCurrentLimit, int peakCurrentLimit) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(false)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(smartCurrentLimit)
				.secondaryCurrentLimit(peakCurrentLimit)
				.voltageCompensation(12);
		m_sparkMaxFrontRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_sparkMaxFrontLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_sparkMaxBackRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_sparkMaxBackLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	/**
	 * Returns drive encoder distance in meters traveled.
	 * 
	 * @return The position
	 */
	public double getDriveEncoderPosition() {
		return m_driveEncoder.getPosition() * kMetersPerMotorRotations;
	}

	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	public double getDriveCurrent() {
		return m_driveMotor.getOutputCurrent();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveEncoder.setPosition(0);
	}

	/**
	 * Gets the current drive motor voltage.
	 * 
	 * @return The motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_driveMotor.getAppliedOutput() * 12;
	}

	/**
	 * Gets the current drive motor temperature.
	 * 
	 * @return The temperature in degrees celcius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getMotorTemperature();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return The module angle
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Returns the module position.
	 * 
	 * @return The module position
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Gets the module speed and angle.
	 * 
	 * @return The module state
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle.
	 * 
	 * @param state The module state
	 */
	public void setModuleState(SwerveModuleState state) {
		double power = state.speedMetersPerSecond;
		double currVoltage = RobotController.getBatteryVoltage();
		if (currVoltage < 7) {
			power *= 0.1;
		} else if (currVoltage < 8) {
			power *= 0.2;
		} else if (currVoltage < 9) {
			power *= 0.4;
		} else if (currVoltage < 10) {
			power *= 0.8;
		}
		m_driveMotor.setVoltage(power);
		double turnPower = m_PIDController.calculate(getModuleAngle(), state.angle.getDegrees());
		m_steerMotor.setVoltage(turnPower);
		if (RobotBase.isSimulation()) {
			m_driveMotorModel.setInputVoltage(power);
			m_driveMotorModel.update(0.02);
			m_driveMotor.getEncoder().setPosition(m_driveMotorModel.getAngularPositionRotations());
			var encoderSimState = m_CANCoder.getSimState();
			m_steerMotorModel.setInputVoltage(turnPower);
			m_steerMotorModel.update(0.02);
			encoderSimState.setRawPosition(m_steerMotorModel.getAngularPositionRotations());
			encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocityRPM());
		}
	}

	/**
	 * Sets the module angle.
	 * 
	 * @param angle
	 */
	public void setAngle(double angle) {
		var out = m_PIDController.calculate(getModuleAngle(), angle);
		SmartDashboard.putNumber("PID out" + m_driveMotor.getDeviceId(), out);
		m_steerMotor.setVoltage(out);
	}
}