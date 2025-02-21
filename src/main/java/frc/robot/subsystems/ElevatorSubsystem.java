// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {
	private final SparkMax m_elevatorMotor = new SparkMax(kElevatorMotorPort, MotorType.kBrushless);
	private final RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();
	private final SparkClosedLoopController m_closedLoopController = m_elevatorMotor.getClosedLoopController();

	private final Timer m_timer = new Timer();
	private final ElevatorFeedforward m_ff = new ElevatorFeedforward(kS, kG, kV, kA);
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAccel));
	// Adjust ramp rate, step voltage, and timeout to make sure elevator doesn't
	// break
	private final SysIdRoutine m_sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(Volts.of(2.5).div(Seconds.of(1)), Volts.of(3), Seconds.of(3)),
			new SysIdRoutine.Mechanism(m_elevatorMotor::setVoltage, null, this));
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
		config.encoder.positionConversionFactor(kMetersPerMotorRotation)
				.velocityConversionFactor(kMetersPerMotorRotation / 60);
		m_elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		resetEncoder();
		if (RobotBase.isSimulation()) {
			m_elevatorMotorSim = new SparkMaxSim(m_elevatorMotor, DCMotor.getNEO(1));
			m_elevatorModel = new ElevatorSim(DCMotor.getNEO(1), kGearRatio, Units.lbsToKilograms(20),
					kMetersPerPulleyRotation / (2 * Math.PI), 0,
					Units.inchesToMeters(90), true, 0);
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
	 * @param arbFF Feedforward voltage
	 */
	public void setPosition(double position, double arbFF) {
		m_setPosition = position;
		m_closedLoopController
				.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF, ArbFFUnits.kVoltage);
	}

	/**
	 * Resets the encoder value to 0 (starting position)
	 */
	public void resetEncoder() {
		m_elevatorEncoder.setPosition(0);
		setPosition(0, 0);
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

	/**
	 * Using Trapezoid Profile to set the position of the elevator
	 * 
	 * @param level the level we want to go to
	 * @return the command
	 */
	private Command goToLevel(double level) {
		var initial = new TrapezoidProfile.State();
		var finalState = new TrapezoidProfile.State(level, 0);
		return startRun(() -> {
			m_timer.restart();
			initial.position = getPosition();
		}, () -> {
			double time = m_timer.get();
			double currentVelocity = m_profile.calculate(time, initial, finalState).velocity;
			double nextVelocity = m_profile.calculate(time + 0.02, initial, finalState).velocity;
			setPosition(level, m_ff.calculateWithVelocities(currentVelocity, nextVelocity));
		});
	}

	/**
	 * Allows the operator to manually move the Elevator for adjustment
	 * 
	 * @param joystick Input from operator's left joystick Y-values
	 * @return Command for moving
	 */
	public Command manualMove(DoubleSupplier joystick) {
		return run(() -> {
			double input = joystick.getAsDouble();
			double speed = Math.signum(input) * Math.pow(input, 2);
			setSpeed(speed * 0.5);
		}).withName("Manual Elevator");
	}

	/**
	 * Moves the elevator to the level one position
	 * 
	 * @return the command
	 */
	public Command goToLevelOneHeight() {
		return goToLevel(kLevelOneHeight).withName("Elevator to Level 1");
	}

	/**
	 * Moves the elevator to the level two positon
	 * 
	 * @return the command
	 */
	public Command goToLevelTwoHeight() {
		return goToLevel(kLevelTwoHeight).withName("Elevator to Level 2");
	}

	/**
	 * Moves the elevator to the level three position
	 * 
	 * @return the command
	 */
	public Command goToLevelThreeHeight() {
		return goToLevel(kLevelThreeHeight).withName("Elevator to Level 2");
	}

	/**
	 * Moves the elevator to the level four position
	 * 
	 * @return the command
	 */
	public Command goToLevelFourHeight() {
		return goToLevel(kLevelFourHeight).withName("Elevator to Level 2");
	}

	/**
	 * Moves the elevator to the coral station position
	 * 
	 * @return the command
	 */
	public Command goToCoralStationHeight() {
		return goToLevel(kCoralStationHeight).withName("Elevator to Coral Station");
	}

	/**
	 * Moves the elevator to the base height positon
	 * 
	 * @return
	 */
	public Command goToBaseHeight() {
		return goToLevel(0).withName("Go to Base Height");
	}

	/**
	 * The other heights when scoring go to a higher height than what is needed to
	 * score so this lowers it to the proper height
	 * 
	 * @return the command
	 */
	public Command lowerToScore() {
		var initial = new TrapezoidProfile.State();
		var finalState = new TrapezoidProfile.State();
		return startRun(() -> {
			m_timer.restart();
			initial.position = getPosition();
			finalState.position = getPosition() - kToScoreHeightDecrease;
		}, () -> {
			double time = m_timer.get();
			double currentVelocity = m_profile.calculate(time, initial, finalState).velocity;
			double nextVelocity = m_profile.calculate(time + 0.02, initial, finalState).velocity;
			setPosition(finalState.position, m_ff.calculateWithVelocities(currentVelocity, nextVelocity));
		}).withName("Lower Elevator to Score");
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
}