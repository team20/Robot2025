package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	SparkMax m_motor;

	public ClimberSubsystem() {
		m_motor = new SparkMax(kClimberMotorPort, MotorType.kBrushless);
	}

	public Command moveForward() {
		return run(() -> {
			m_motor.set(kSpeed);
		}).withName("Climber Forwards");
	}

	public Command moveBackward() {
		return run(() -> {
			m_motor.set(-kSpeed);
		}).withName("Climber Backwards");
	}
}