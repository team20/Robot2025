// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorSimulator extends SimpleElevatorSubsystem {
	private final SparkMaxSim m_elevatorMotorSim;
	private final ElevatorSim m_elevatorModel;
	private final MechanismLigament2d m_elevatorLigament = new MechanismLigament2d("elevator", Units.inchesToMeters(36),
			90, 10, new Color8Bit(Color.kYellow));
	private final MechanismLigament2d m_wristMount = m_elevatorLigament
			.append(
					new MechanismLigament2d("wristMountPoint", Units.inchesToMeters(5), 180, 10,
							new Color8Bit(Color.kYellow)))
			.append(
					new MechanismLigament2d("wristMount", Units.inchesToMeters(8), 90, 10,
							new Color8Bit(Color.kBlack)));

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSimulator(MechanismRoot2d root) {
		super();
		root.append(m_elevatorLigament);
		m_elevatorMotorSim = new SparkMaxSim(m_elevatorMotor, DCMotor.getNEO(1));
		m_elevatorModel = new ElevatorSim(DCMotor.getNEO(1), kGearRatio, Units.lbsToKilograms(20),
				kMetersPerPulleyRotation / (2 * Math.PI), 0,
				Units.inchesToMeters(90), true, 0);
	}

	public MechanismLigament2d getWristMount() {
		return m_wristMount;
	}

	@Override
	public void simulationPeriodic() {
		m_elevatorModel.setInputVoltage(m_elevatorMotorSim.getAppliedOutput() * 12);
		m_elevatorModel.update(0.02);
		m_elevatorMotorSim.iterate(m_elevatorModel.getVelocityMetersPerSecond(), 12, 0.02);
		m_elevatorMotorSim.setPosition(m_elevatorModel.getPositionMeters());
		m_elevatorMotorSim.setVelocity(m_elevatorModel.getVelocityMetersPerSecond());
	}

	@Override
	public void periodic() {
		super.periodic();
		m_elevatorLigament.setLength(Units.inchesToMeters(24) + getPosition());
	}

}