package frc.robot.subsystems;

import static frc.robot.Constants.CheeseStickConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CheeseStickSubsystem extends SubsystemBase {
	private final Servo m_servo = new Servo(kServoPort);

	public CheeseStickSubsystem() {
		// https://docs.revrobotics.com/rev-crossover-products/servo/srs#electrical-specifications
		m_servo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
		m_servo.setZeroLatch();
	}

	@Override
	public void periodic() {
	}

	/**
	 * Returns a command to command the servo to rotate to the left. Does not wait
	 * for it to finish.
	 * 
	 * @return The command.
	 */
	public Command goLeft() {
		return runOnce(() -> m_servo.set(0)).withName("Servo Go Left");
	}

	/**
	 * Returns a command to command the servo to rotate to the right. Does not wait
	 * for it to finish.
	 * 
	 * @return The command.
	 */
	public Command goRight() {
		return runOnce(() -> m_servo.set(1)).withName("Servo Go Right");
	}
}