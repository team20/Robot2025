// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeGrabberSubsystem extends SubsystemBase {

	private final SparkMax m_flywheelLeft;
	private final SparkMax m_flywheelRight;
	private final SparkClosedLoopController m_controllerLeft;
	private final SparkClosedLoopController m_controllerRight;

	private final RelativeEncoder m_encoderLeft;
	private final RelativeEncoder m_encoderRight;
	private double m_setVelocity;

	public AlgaeGrabberSubsystem() {
		SparkMaxConfig config = new SparkMaxConfig();

		m_flywheelLeft = new SparkMax(AlgaeConstants.kLeftPort, MotorType.kBrushless);
		m_flywheelRight = new SparkMax(AlgaeConstants.kRightPort, MotorType.kBrushless);
		m_controllerLeft = m_flywheelLeft.getClosedLoopController();
		m_controllerRight = m_flywheelRight.getClosedLoopController();
		m_encoderLeft = m_flywheelLeft.getEncoder();
		m_encoderRight = m_flywheelRight.getEncoder();

		// Initialize Motors
		m_flywheelLeft.restoreFactoryDefaults();
		m_flywheelLeft.setInverted(kBottomInvert);
		m_flywheelLeft.setIdleMode(IdleMode.kCoast);
		m_flywheelLeft.enableVoltageCompensation(12);
		m_flywheelLeft.setSmartCurrentLimit(kSmartCurrentLimit);
		m_flywheelLeft.setSecondaryCurrentLimit(
				kPeakCurrentLimit,
				kPeakCurrentDurationMillis);

		m_flywheelRight.restoreFactoryDefaults();
		m_flywheelRight.setInverted(kTopInvert);
		m_flywheelRight.setIdleMode(IdleMode.kCoast);
		m_flywheelRight.enableVoltageCompensation(12);
		m_flywheelRight.setSmartCurrentLimit(kSmartCurrentLimit);
		m_flywheelRight.setSecondaryCurrentLimit(
				kPeakCurrentLimit,
				kPeakCurrentDurationMillis);

		m_encoderLeft.setVelocityConversionFactor(kGearRatio);
		m_controllerLeft.setP(kP);
		m_controllerLeft.setI(kI);
		m_controllerLeft.setD(kD);
		m_controllerLeft.setIZone(kIz);
		m_controllerLeft.setFF(kFF);
		m_controllerLeft.setOutputRange(kMinOutput, kMaxOutput);

		m_encoderRight.setVelocityConversionFactor(kGearRatio);
		m_controllerRight.setP(kP);
		m_controllerRight.setI(kI);
		m_controllerRight.setD(kD);
		m_controllerRight.setIZone(kIz);
		m_controllerRight.setFF(kFF);
		m_controllerRight.setOutputRange(kMinOutput, kMaxOutput);
	}

	@Override
	public void periodic() {
	}
}
