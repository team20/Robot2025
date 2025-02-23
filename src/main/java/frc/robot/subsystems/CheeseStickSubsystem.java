package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CheeseStickConstants.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CheeseStickSubsystem extends SubsystemBase {
	private final Servo m_servo = new Servo(kServoPort);
	private final PWMSim m_sim;
	private final MechanismLigament2d m_ligament = new MechanismLigament2d("Cheeese Stick", 0, 90);

	public CheeseStickSubsystem(MechanismObject2d attachment) {
		// https://docs.revrobotics.com/rev-crossover-products/servo/srs#electrical-specifications
		m_servo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
		m_servo.setZeroLatch();
		attachment.append(m_ligament);
		if (RobotBase.isSimulation()) {
			m_sim = new PWMSim(m_servo);
		} else {
			m_sim = null;
		}
	}

	/**
	 * Logs data to the smart dashboard. Displays the simulated mechanism.
	 */
	@Override
	public void simulationPeriodic() {
		double position = m_sim.getPosition();
		SmartDashboard.putNumber("Cheese Stick Position", position);
		m_ligament.setLength(kExtensionLength.in(Meters) * position);
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
}