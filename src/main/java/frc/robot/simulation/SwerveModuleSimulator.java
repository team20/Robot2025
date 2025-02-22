// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.SwerveModule;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModuleSimulator extends SwerveModule {
	private final SparkFlexSim m_steerMotorSim;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;

	public SwerveModuleSimulator(int canId, int drivePort, int steerPort) {
		super(canId, drivePort, steerPort);
		m_steerMotorSim = new SparkFlexSim(m_steerMotor, DCMotor.getNEO(1));
		m_driveMotorModel = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
				DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio));
		m_steerMotorModel = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
				DCMotor.getKrakenX60(1));
	}

	/**
	 * Sets the drive motor speeds and module angle of this
	 * {@code SwerveModuleSimulator}.
	 * 
	 * @param state a {@code SwerveModuleState} containing the target speeds and
	 *        angle
	 */
	@Override
	public void setModuleState(SwerveModuleState state) {
		super.setModuleState(state);
		update();
	}

	/**
	 * Updates this {@code SwerveModuleSimulator}.
	 */
	private void update() {
		var driveMotorState = m_driveMotor.getSimState();
		m_driveMotorModel.setInputVoltage(driveMotorState.getMotorVoltage());
		m_driveMotorModel.update(0.02);
		driveMotorState.setRotorVelocity(m_driveMotorModel.getAngularVelocityRPM() / 60);
		driveMotorState.setRawRotorPosition(m_driveMotorModel.getAngularPositionRotations());

		m_steerMotorModel.setInputVoltage(m_steerMotorSim.getAppliedOutput() * 12);
		m_steerMotorModel.update(0.02);
		m_steerMotorSim.iterate(m_steerMotorModel.getAngularVelocityRPM(), 12, 0.02);
		var encoderSimState = m_CANCoder.getSimState();
		encoderSimState.setRawPosition(m_steerMotorModel.getAngularPositionRotations() / kSteerGearRatio);
		encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocityRPM() / 60 / kSteerGearRatio);
	}
}
