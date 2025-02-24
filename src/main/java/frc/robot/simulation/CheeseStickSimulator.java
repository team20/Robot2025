package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CheeseStickConstants.*;

import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CheeseStickSimulator extends SimpleCheeseStickSubsystem {
	private final PWMSim m_sim;
	private final MechanismLigament2d m_ligament = new MechanismLigament2d("Cheeese Stick", 0, 90);

	public CheeseStickSimulator(MechanismObject2d attachment) {
		super();
		attachment.append(m_ligament);
		m_sim = new PWMSim(m_servo);
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

}