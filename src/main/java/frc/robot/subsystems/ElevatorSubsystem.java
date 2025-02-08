// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: Setpoint and hold?

public class ElevatorSubsystem extends SubsystemBase {
	private final SparkMax m_elevatorMotor = new SparkMax(kElevatorMotorPort, MotorType.kBrushless);
	private final RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();
	private final SparkClosedLoopController m_closedLoopController = m_elevatorMotor.getClosedLoopController();

	private double m_setPosition = 0;

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
	public ElevatorSubsystem(MechanismRoot2d root) {
		root.append(m_elevatorLigament);
		var config = new SparkMaxConfig();
		config
				.idleMode(IdleMode.kBrake) // TODO: Soft limits?
				.smartCurrentLimit(kSmartCurrentLimit)
				.secondaryCurrentLimit(kSecondaryCurrentLimit)
				.voltageCompensation(12);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(kP, kI, kD);
		m_elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		resetEncoder();
		if (RobotBase.isSimulation()) {
			m_elevatorMotorSim = new SparkMaxSim(m_elevatorMotor, DCMotor.getNEO(1));
			m_elevatorModel = new ElevatorSim(DCMotor.getNEO(1), 10, Units.lbsToKilograms(20),
					Units.inchesToMeters(2), 0, Units.inchesToMeters(90), true, 0);
		} else {
			m_elevatorMotorSim = null;
			m_elevatorModel = null;
		}
	}

	public MechanismLigament2d getWristMount() {
		return m_wristMount;
	}

	/**
	 * This method sets the speed of the motor
	 * 
	 * @param speed the speed you want to set it to
	 */
	public void setSpeed(double speed) {
		m_elevatorMotor.set(speed);
	}

	/**
	 * Gets the position
	 * 
	 * @return the position of the motor (through the encoder)
	 */
	public double getPosition() {
		return m_elevatorEncoder.getPosition();
	}

	/**
	 * Tests if within the tolerance of the setpoint of the motor
	 * 
	 * @return true if at the setpoint
	 */
	public boolean atSetpoint() {
		return (Math.abs(m_setPosition - getPosition()) <= kTolerance);
	}

	/**
	 * Sets the position of the motor (using PID)
	 * 
	 * @param position Position to set the master motor
	 */
	public void setPosition(double position) {
		m_setPosition = position;
		m_closedLoopController.setReference(position, ControlType.kPosition);
	}

	/**
	 * Resets the encoder value to 0 (starting position)
	 */
	public void resetEncoder() {
		m_elevatorEncoder.setPosition(0);
		setPosition(0);
	}

	/**
	 * Determine the output current of the motor
	 * 
	 * @return The max value that the output current gets to`
	 */
	public double getOutputCurrent() {
		return Math.abs(m_elevatorMotor.getOutputCurrent());
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
		m_elevatorLigament.setLength(Units.inchesToMeters(24) + getPosition());
		// This method will be called once per scheduler run
	}

	/**
	 * Stops the motor
	 * 
	 * @return Sets the speed to 0
	 */
	public Command stopMotor() {
		return runOnce(() -> setSpeed(0)).withName("Elevator motor stop");
	}

	/**
	 * Reverse the motor by setting the speeed to negative
	 * 
	 * @return the command to set the speed
	 */
	public Command reverseMotor() {
		return runOnce(() -> setSpeed(-1)).withName("Elevator motor backwards");
	}

	/**
	 * Make the motor spin forward with a positive speed
	 * 
	 * @return the command to set the speed
	 */
	public Command forwardMotor() {
		return runOnce(() -> setSpeed(1)).withName("Elevator motor forward");
	}

	public Command goToLevelOneHeight() {
		return runOnce(() -> setPosition(kLevelOneHeight)).withName("Elevator to Level 1");
	}

	public Command goToLevelTwoHeight() {
		return runOnce(() -> setPosition(kLevelTwoHeight)).withName("Elevator to Level 2");
	}

	public Command goToLevelThreeHeight() {
		return runOnce(() -> setPosition(kLevelThreeHeight)).withName("Elevator to Level 3");
	}

	public Command goToLevelFourHeight() {
		return runOnce(() -> setPosition(kLevelFourHeight)).withName("Elevator to Level 4");
	}

	public Command goToCoralStationHeight() {
		return runOnce(() -> setPosition(kCoralStationHeight)).withName("Elevator to Coral Station");
	}

	public Command goToBaseHeight() {
		return runOnce(() -> setPosition(0)).withName("Go to Base Height");
	}

	public Command lowerToScore() {
		return runOnce(() -> setPosition(getPosition() - kToScoreHeightDecrease)).withName("Lower Elevator to Score");
	}

}