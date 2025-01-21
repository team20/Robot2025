package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimationSubsystem extends SubsystemBase {

	/**
	 * The {@code PhotonCamera} used by this {@code PoseEstimationSubsystem}.
	 */
	private final PhotonCamera m_camera;

	/**
	 * The {@code PhotonCamera} used by this {@code PoseEstimationSubsystem}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The standard deviations of model states (smaller values will cause the Kalman
	 * filter to more trust the estimates).
	 */
	private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

	/**
	 * The standard deviations of the vision measurements (smaller values will cause
	 * the Kalman filter to more trust the measurements).
	 */
	private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

	/**
	 * The {@code PhotonPoseEstimator} used by this {@code PoseEstimationSubsystem}.
	 */
	private final PhotonPoseEstimator m_photonPoseEstimator;

	/**
	 * The {@code SwerveDrivePoseEstimator} used by this
	 * {@code PoseEstimationSubsystem}.
	 */
	private final SwerveDrivePoseEstimator m_poseEstimator;

	/**
	 * The {@code StructPublisher} for reporting the detected {@code Pose2d} of the
	 * robot.
	 */
	private final StructPublisher<Pose2d> m_detectedPosePublisher;

	/**
	 * The {@code StructPublisher} for reporting the estimated {@code Pose2d} of the
	 * robot.
	 */
	private final StructPublisher<Pose2d> m_estimatedPosePublisher;

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PoseEstimationSubsystem} (found in the PhotonVision UI)
	 * @param driveSubsystem {@code DriveSubsystem} to be used by the
	 *        {@code PoseEstimationSubsystem}
	 */
	public PoseEstimationSubsystem(String cameraName, DriveSubsystem driveSubsystem) {
		m_camera = RobotBase.isSimulation()
				? new PhotonCameraSimulator(cameraName, driveSubsystem, new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
						0.01, 3,
						0.1)
				: new PhotonCamera(cameraName);
		this.m_driveSubsystem = driveSubsystem;
		m_photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamera);
		m_poseEstimator = new SwerveDrivePoseEstimator(
				driveSubsystem.kinematics(),
				driveSubsystem.getHeading(),
				driveSubsystem.getModulePositions(),
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs);
		m_detectedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonCamera", Pose2d.struct)
				.publish();
		m_estimatedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PoseEstimationSubsystem", Pose2d.struct)
				.publish();
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		for (var r : m_camera.getAllUnreadResults()) {
			var e = m_photonPoseEstimator.update(r);
			if (e.isPresent()) {
				var v = e.get();
				m_poseEstimator.addVisionMeasurement(v.estimatedPose.toPose2d(), v.timestampSeconds);
				m_detectedPosePublisher.set(v.estimatedPose.toPose2d());
			}
		}
		m_poseEstimator.update(m_driveSubsystem.getHeading(), m_driveSubsystem.getModulePositions());
		m_estimatedPosePublisher.set(m_poseEstimator.getEstimatedPosition());
	}

	/**
	 * Returns the most recent estimated {@code Pose2d} of the robot.
	 * 
	 * @return the most recent estimated {@code Pose2d} of the robot
	 */
	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}
}