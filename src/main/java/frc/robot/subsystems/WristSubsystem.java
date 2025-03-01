// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.WristConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class WristSubsystem extends SubsystemBase {
	private final SparkMax m_wristMotor = new SparkMax(kWristMotorPort, MotorType.kBrushless);
	private final SparkAbsoluteEncoder m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder();
	private final SparkClosedLoopController m_wristClosedLoopController = m_wristMotor.getClosedLoopController();

	// Adjust ramp rate, step voltage, and timeout to make sure wrist doesn't break
	private final SysIdRoutine m_sysidRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(Volts.of(2.5).div(Seconds.of(1)), Volts.of(3), Seconds.of(3)),
			new SysIdRoutine.Mechanism(m_wristMotor::setVoltage, null, this));

	private final SparkMaxSim m_wristSim;
	private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;
	private final SingleJointedArmSim m_wristModel;
	private final MechanismLigament2d m_wrist = new MechanismLigament2d("wrist", Units.inchesToMeters(9), 0);

	/** Creates a new WristSubsystem. */
	public WristSubsystem(MechanismLigament2d wristMount) {
		wristMount.append(m_wrist);
		var config = new SparkMaxConfig();
		config
				.inverted(true)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(kSmartCurrentLimit)
				.secondaryCurrentLimit(kSecondaryCurrentLimit)
				.voltageCompensation(12);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.pid(kP, kI, kD);
		config.softLimit.forwardSoftLimit(kWristForwardSoftLimit).forwardSoftLimitEnabled(true);
		config.softLimit.reverseSoftLimit(kWristReverseSoftLimit).reverseSoftLimitEnabled(true);
		config.absoluteEncoder.zeroOffset(kWristOffset).positionConversionFactor(360);
		m_wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		if (RobotBase.isSimulation()) {
			m_wristSim = new SparkMaxSim(m_wristMotor, DCMotor.getNEO(1));
			m_absoluteEncoderSim = new SparkAbsoluteEncoderSim(m_wristMotor);
			m_wristModel = new SingleJointedArmSim(DCMotor.getNEO(1), 5, 2, 0.1, 0,
					Math.PI, false, 0);
		} else {
			m_wristSim = null;
			m_absoluteEncoderSim = null;
			m_wristModel = null;
		}
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
		return m_absoluteEncoder.getPosition();
	}

	/**
	 * Tests if within the tolerance of the angle of the motor
	 * 
	 * @return true if at the setpoint
	 */
	public Trigger atAngle(double target) {
		return new Trigger(() -> Math.abs(target - getAngle()) <= kTolerance);
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
		m_wristSim.setPosition(Math.toDegrees(m_wristModel.getAngleRads()));
		m_absoluteEncoderSim.iterate(velocityRPM, 0.02);
		m_absoluteEncoderSim.setPosition(Math.toDegrees(m_wristModel.getAngleRads()));
	}

	@Override
	public void periodic() {
		// Negate to make angle CCW+, subtract 180 to get 0 degrees in the right place
		double angle = RobotBase.isReal() ? getAngle() : m_absoluteEncoderSim.getPosition();
		SmartDashboard.putNumber("Wrist/Current Angle", angle);
		m_wrist.setAngle(-angle - 180);
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
	 * Reverse the motor by setting the speed to negative
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
			m_wristMotor.set(speed * 0.5);
		}).withName("Manual Wrist");
	}

	/**
	 * Sets the angle of the motors
	 *
	 * @param position angle (in degrees) to set the wrist to
	 */
	public Command goToAngle(double angle) {
		return run(() -> {
			SmartDashboard.putNumber("Wrist/Target Angle", angle);
			m_wristClosedLoopController.setReference(angle, ControlType.kPosition);
		}).until(atAngle(angle)).withName("Wrist go to angle");
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