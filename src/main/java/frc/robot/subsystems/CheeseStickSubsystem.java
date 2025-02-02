package frc.robot.subsystems;

import static frc.robot.Constants.CheeseStickConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CheeseStickSubsystem extends SubsystemBase {
	private Servo m_motor;
	public Timer time;

	public CheeseStickSubsystem() {
		m_motor = new Servo(kServoPort);
		time = new Timer();
		m_motor.setBoundsMicroseconds(2500, 0, 1500, 0, 500);
	}

	public void init() {
		m_motor.setZeroLatch();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Time", time.get());
		SmartDashboard.putNumber("Position", m_motor.getPosition());
	}

	/**
	 * Rotates the servo. Does not wait for it to finish.
	 * 
	 * @param angle The angle from 0 in the left to 270 in the right.
	 * @return The command.
	 */
	public Command rotateCommand(Double angle) {
		if (angle == null)
			return runOnce(() -> m_motor.set(.5));
		return runOnce(() -> m_motor.set(angle / kMaxRotation)).withName("Rotate Servo");
	}
}