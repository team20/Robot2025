package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	private final SparkMax m_motor = new SparkMax(kClimberMotorPort, MotorType.kBrushless);

	public ClimberSubsystem() {
		var config = new SparkMaxConfig();
		config.smartCurrentLimit(kSmartCurrentLimit).secondaryCurrentLimit(kSecondaryCurrentLimit);
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	/**
	 * Spins the motor forward until changed
	 * 
	 * @return forward command
	 */
	public Command moveForward() {
		return runOnce(() -> {
			m_motor.set(kSpeed);
		}).withName("Climber Forwards");
	}

	/**
	 * Spins the motor backwards until changed
	 * 
	 * @return backwards command
	 */
	public Command moveBackward() { // TODO: might break things?
		return run(() -> {
			m_motor.set(-kSpeed);
		}).withName("Climber Backwards");
	}
}