package frc.robot.subsystems;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.CheeseStickConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is for the cheese stick gripper.
 */
public class CheeseStickSubsystem extends SubsystemBase {
	private SparkMax m_motor;
	private RelativeEncoder m_encoder;
	private DoubleSupplier m_position;

	public CheeseStickSubsystem() {
		m_motor = new SparkMax(kMotorPort, kBrushless);
		m_motor.configure(kMotorConfig, kResetSafeParameters, kNoPersistParameters);
		m_encoder = m_motor.getEncoder();
		m_position = () -> m_encoder.getPosition();
	}

	/**
	 * Resets the subsystem to the default state based on the springs.
	 * 
	 * @return The command.
	 */
	public Command resetPositionCommand() {
		return runOnce(
				() -> m_motor.configure(kMotorConfig.idleMode(kCoast), kResetSafeParameters, kNoPersistParameters))
						.andThen(waitTime(kResetLength)).andThen(() -> {
							m_motor.configure(
									kMotorConfig.idleMode(kBrake), kResetSafeParameters, kNoPersistParameters);
							m_encoder.setPosition(0);
						});
	}

	/**
	 * Resturns a supplier for the cheese stick position. This might be useful in
	 * the future.
	 * 
	 * @return The supplier.
	 */
	public DoubleSupplier getPosition() {
		return m_position;
	}

	/**
	 * Creates a command which opens the cheese stick in order to insert into the
	 * coral.
	 * 
	 * @return The command.
	 */
	public Command createOpenCommand() {
		return createMoveCommand(kOpenDistance);
	}

	/**
	 * Creates a command which grips the coral.
	 * 
	 * @return The command.
	 */
	public Command createGripCommand() {
		return createMoveCommand(0);
	}

	/**
	 * Creates a command which moves the cheese stick to the specified position.
	 * 
	 * @param distance The position to go to.
	 * @return The command.
	 */
	private Command createMoveCommand(double distance) {
		PIDController PID = new PIDController(kP, 0, 0);
		PID.setTolerance(kTolerance);
		return new Command() {
			/**
			 * Evaluate the PID controller.
			 */
			public void execute() {
				m_motor.set(PID.calculate(m_position.getAsDouble()));
			}

			/**
			 * Checks if the position is within tolerance or if the timeout has ocurred.
			 */
			public boolean isFinished() {
				return PID.atSetpoint();
			}

			/**
			 * Stops the motor from moving.
			 */
			public void end(boolean interrupted) {
				m_motor.set(0);
				PID.close();
			}
		}.withTimeout(kGripTimeout);
	}

	/**
	 * Creates a command which controls the cheese stick based on an activator
	 * (likely a button)
	 * 
	 * @param activator The supplier which defines when the cheese stick is open.
	 * @return The command.
	 */
	public Command createManualCommand(BooleanSupplier activator) {
		Command open = createOpenCommand();
		Command grip = createGripCommand();
		return new Command() {
			public void execute() {
				if (activator.getAsBoolean()) {
					if (!open.isScheduled())
						open.schedule();
					if (grip.isScheduled())
						grip.cancel();
				} else {
					if (open.isScheduled())
						open.cancel();
					if (!grip.isScheduled())
						grip.schedule();
				}
			}
		};
	}
}
