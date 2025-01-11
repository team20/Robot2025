// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
	/** Creates a new MotorSubsystem. */
	private final SparkMax m_SparkMaxOne = new SparkMax(1, MotorType.kBrushless);
	private final RelativeEncoder m_encoder = m_SparkMaxOne.getEncoder();

	public MotorSubsystem() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(false).idleMode(IdleMode.kBrake);
		m_SparkMaxOne.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void setSpeed(double speed) {
		m_SparkMaxOne.set(speed);
	}

	public Command reverseMotor() {
		return runOnce(() -> setSpeed(-1));
	}

	public Command forwardMotor() {
		return runOnce(() -> setSpeed(1));
	}

	public Command stopMotor() {
		return runOnce(() -> setSpeed(0));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public Command spinMotorCommand() {
		return runEnd(() -> setSpeed(0.1), () -> setSpeed(0)).withTimeout(10);
	}
}