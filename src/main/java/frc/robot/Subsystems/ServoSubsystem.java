package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
	private Servo m_motor;
	public Timer time;

	public ServoSubsystem() {
		m_motor = new Servo(0);
		time = new Timer();
	}

	public void init() {
		m_motor.setZeroLatch();
	}

	public void run() {
		// TODO Auto-generated method stub
		super.periodic();

		time.start();
		SmartDashboard.putNumber("Time", time.get());
		SmartDashboard.putNumber("Position", m_motor.getPosition());

		m_motor.set(1.0);
	}
}
