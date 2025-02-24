// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristSimulator extends SimpleWristSubsystem {

	private final SparkMaxSim m_wristSim;
	private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;
	private final SingleJointedArmSim m_wristModel;
	private final MechanismLigament2d m_wrist = new MechanismLigament2d("wrist", Units.inchesToMeters(9), 90);

	/** Creates a new WristSubsystem. */
	public WristSimulator(MechanismLigament2d wristMount) {
		super();
		m_wristSim = new SparkMaxSim(m_wristMotor, DCMotor.getNEO(1));
		m_absoluteEncoderSim = new SparkAbsoluteEncoderSim(m_wristMotor);
		m_wristModel = new SingleJointedArmSim(DCMotor.getNEO(1), 5, 2, 0.1, 0,
				Math.PI, false, 0);
	}

	/**
	 * Gets the ligament to bind the cheese stick to.
	 * 
	 * @return The ligament.
	 */
	public MechanismObject2d getCheeseStickMount() {
		return m_wrist;
	}

	@Override
	public void simulationPeriodic() {
		m_wristModel.setInputVoltage(m_wristSim.getAppliedOutput() * 12);
		m_wristModel.update(0.02);
		var velocityRPM = m_wristModel.getVelocityRadPerSec() / (2 * Math.PI) / 60;
		m_wristSim.iterate(velocityRPM, 12, 0.02);
		m_wristSim.setPosition(m_wristModel.getAngleRads() / (2 * Math.PI));
		m_absoluteEncoderSim.iterate(velocityRPM, 0.02);
		m_absoluteEncoderSim.setPosition(m_wristModel.getAngleRads() / (2 * Math.PI));
		m_wrist.setAngle(-90 + getAngle());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Output current", getOutputCurrent());
		SmartDashboard.putNumber("Angle", getAngle());
		SmartDashboard.putNumber("Applied output", m_wristMotor.getAppliedOutput());
	}

}