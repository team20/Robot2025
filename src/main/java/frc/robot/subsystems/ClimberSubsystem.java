package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	private final SparkMax m_motor = new SparkMax(kClimberMotorPort, MotorType.kBrushless);
	private final SparkClosedLoopController m_climberClosedLoopController = m_motor
			.getClosedLoopController();
	private final DigitalInput m_leftSensor = new DigitalInput(1);
	private final DigitalInput m_rightSensor = new DigitalInput(2);
	private RelativeEncoder encoder;

	public ClimberSubsystem() {
		var config = new SparkMaxConfig();
		config.smartCurrentLimit(kSmartCurrentLimit).idleMode(IdleMode.kBrake);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(kP, kI, kD);
		encoder = m_motor.getEncoder();
		encoder.setPosition(0);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		if (m_leftSensor.get() && m_rightSensor.get()) {
			SmartDashboard.putString("Climber Color", Color.kLawnGreen.toHexString());
		} else if (m_leftSensor.get() ^ m_rightSensor.get()) { // Left or right, but not both
			SmartDashboard.putString("Climber Color", Color.kYellow.toHexString());
		} else {
			SmartDashboard.putString("Climber Color", Color.kBlack.toHexString());
		}
	}

	/**
	 * Allows the operator to manually move the Climber for adjustment
	 * 
	 * @param joystick Input from operator's left joystick Y-values
	 * @return Command for moving
	 */
	public Command manualMove(DoubleSupplier joystick) {
		return run(() -> {
			double input = joystick.getAsDouble();
			double speed = Math.signum(input) * Math.pow(input, 2);
			m_motor.set(speed * 0.5);
		}).withName("Manual Climber");
	}

	public Command retract() {
		return run(() -> {
			m_climberClosedLoopController.setReference(0, ControlType.kPosition);
		}).until(() -> Math.abs(encoder.getPosition()) < kTolerance).withName("Climber Retract");
	}

	public Command deploy() {
		return run(() -> {
			m_climberClosedLoopController.setReference(-400, ControlType.kPosition);
		}).until(() -> Math.abs(-400 - encoder.getPosition()) < kTolerance).withName("Climber Deploy");
	}
}