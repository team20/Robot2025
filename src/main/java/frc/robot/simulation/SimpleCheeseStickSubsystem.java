package frc.robot.simulation;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.CheeseStickConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SimpleCheeseStickSubsystem extends SubsystemBase {
	protected final Servo m_servo = new Servo(kServoPort);

	public SimpleCheeseStickSubsystem() {
		// https://docs.revrobotics.com/rev-crossover-products/servo/srs#electrical-specifications
		m_servo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
		m_servo.setZeroLatch();
	}

	/**
	 * Returns a command to command the servo to rotate anti-clockwise. This pulls
	 * against springs and retracts the cheese stick. Does not wait
	 * for it to finish.
	 * 
	 * @return The command.
	 */
	public Command release() {
		System.out.println("Servo Release");
		return runOnce(() -> m_servo.set(kReleaseDistance)).withName("Servo Release");
	}

	/**
	 * Returns a command to command the servo to rotate clockwise. This is pushed by
	 * springs and extends the cheese stock. Does not wait
	 * for it to finish.
	 * 
	 * @return The command.
	 */
	public Command grab() {
		return runOnce(() -> m_servo.set(1)).withName("Servo Grab");
	}

	/**
	 * Creates a {@code Command} for testing this {@code SimpleCheeseStickSubsystem}
	 * (grab and release).
	 * 
	 * @param duration the duration of each movement in seconds
	 * 
	 * @return a {@code Command} for testing this {@code SimpleCheeseStickSubsystem}
	 */
	public Command testCommand(double duration) {
		return sequence(grab(), new WaitCommand(duration), release(), new WaitCommand(duration));
	}

}