package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Operates the Robot's ability to detect and identify AprilTags using LimeLight
 * cameras.
 * 
 * @author Jonathan Waters
 * @author Andrew Hwang
 * @author Jamis Orr
 */
public class AprilTagSubsystem extends SubsystemBase {
	public double m_x, m_y, m_z, m_pitch, m_yaw, m_roll, m_xT, m_yT, m_zT, m_pitchT, m_yawT, m_rollT;

	/**
	 * Instantiates the {@code NetworkTable} in {@code AprilTagSubsystem}.
	 */
	NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
	private static AprilTagSubsystem s_subsystem;

	private MedianFilter m_filterX = new MedianFilter(10);
	private MedianFilter m_filterY = new MedianFilter(10);
	private MedianFilter m_filterZ = new MedianFilter(10);
	private MedianFilter m_filterPitch = new MedianFilter(10);
	private MedianFilter m_filterRoll = new MedianFilter(10);
	private MedianFilter m_filterYaw = new MedianFilter(10);
	private MedianFilter m_filterXT = new MedianFilter(10);
	private MedianFilter m_filterYT = new MedianFilter(10);
	private MedianFilter m_filterZT = new MedianFilter(10);
	private MedianFilter m_filterPitchT = new MedianFilter(10);
	private MedianFilter m_filterRollT = new MedianFilter(10);
	private MedianFilter m_filterYawT = new MedianFilter(10);
	private boolean m_tagInView;

	private double m_tX;

	/** Creates a new AprilTagSubsystem. */
	public AprilTagSubsystem() { // constructor, makes the apriltagSubsystem = to the first instance called
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("AprilTag subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
	}

	public static AprilTagSubsystem get() { // returns the first instance called, guarantees that there's not
											// conflicting instances
		return s_subsystem;
	}

	/**
	 * A method run periodically (every 20 ms).
	 */
	@Override
	public void periodic() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry robottran = table.getEntry("botpose_targetspace");
		NetworkTableEntry targettran = table.getEntry("targetpose_robotspace");
		NetworkTableEntry tx = table.getEntry("tx");
		// read values periodically
		double[] translationRobot = robottran.getDoubleArray(new double[6]);
		double[] translationTarget = targettran.getDoubleArray(new double[6]);

		m_tX = tx.getDouble(0);
		if (!translationRobot.equals(new double[6])) {
			m_x = m_filterX.calculate(translationRobot[0]);
			m_y = m_filterY.calculate(translationRobot[1]);
			m_z = m_filterZ.calculate(translationRobot[2]);
			m_pitch = m_filterPitch.calculate(translationRobot[3]);
			m_yaw = m_filterYaw.calculate(translationRobot[4]); // [-71, 66]
			m_roll = m_filterRoll.calculate(translationRobot[5]);
			m_xT = m_filterXT.calculate(translationTarget[0]);
			m_yT = m_filterYT.calculate(translationTarget[1]);
			m_zT = m_filterZT.calculate(translationTarget[2]);
			m_pitchT = m_filterPitchT.calculate(translationTarget[3]);
			m_yawT = m_filterYawT.calculate(translationTarget[4]); // [-71, 66]
			m_rollT = m_filterRollT.calculate(translationTarget[5]);
			m_tagInView = true;
		} else {
			m_tagInView = false;
		}

		// Posts the translation values into SmartDashboard
		SmartDashboard.putNumber("LimelightX", m_x);
		SmartDashboard.putNumber("LimelightY", m_y);
		SmartDashboard.putNumber("LimelightZ", m_z);
		SmartDashboard.putNumber("LimelightPitch", m_pitch);
		SmartDashboard.putNumber("LimelightYaw", m_yaw);
		SmartDashboard.putNumber("LimelightRoll", m_roll);
		SmartDashboard.putBoolean("Tag in View", m_tagInView);

		SmartDashboard.putNumber("LimelightXT", m_xT);
		SmartDashboard.putNumber("LimelightYT", m_yT);
		SmartDashboard.putNumber("LimelightZT", m_zT);
		SmartDashboard.putNumber("LimelightPitchT", m_pitchT);
		SmartDashboard.putNumber("LimelightYawT", m_yawT);
		SmartDashboard.putNumber("LimelightRollT", m_rollT);
	}

	/**
	 * Returns the distance between the camera and the AprilTag
	 * 
	 * @return the distance between the camera and the AprilTag
	 */
	public double getDistance() {
		return m_z;
	}

	/**
	 * Returns the yaw angle between the camera and the AprilTag
	 * 
	 * @return the yaw angle between the camera and the AprilTag
	 */
	public double getYaw() {
		return m_yaw;
	}

	/**
	 * Returns the x (horizontal) translation from the AprilTag
	 * 
	 * @return the x (horizontal) translation from the AprilTag
	 */
	public double getX() {
		return m_x;
	}

	/**
	 * Returns a boolean statement if the AprilTag is within the view of the camera.
	 * 
	 * @return a boolean statement if the AprilTag is within the view of the camera.
	 */
	public boolean tagInView() {
		return m_tagInView;
	}

	public double getTX() {
		return m_tX;
	}
}