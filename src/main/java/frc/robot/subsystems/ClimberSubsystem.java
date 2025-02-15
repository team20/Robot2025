package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	private final SparkMax m_motor;

	public ClimberSubsystem() {
		m_motor = new SparkMax(kClimberMotorPort, MotorType.kBrushless);
	}

	/**
	 * Spins the motor forward until changed
	 * 
	 * @return forward command
	 */
	public Command moveForward() {
		return run(() -> {
			m_motor.set(kSpeed);
		}).withName("Climber Forwards");
	}

	/**
	 * Spins the motor backwards until changed
	 * 
	 * @return backwards command
	 */
	public Command moveBackward() {
		return run(() -> {
			m_motor.set(-kSpeed);
		}).withName("Climber Backwards");
	}
}