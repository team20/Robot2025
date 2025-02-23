package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

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