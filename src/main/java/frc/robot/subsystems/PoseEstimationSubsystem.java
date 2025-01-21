package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
	 * The {@code SwerveDrivePoseEstimator} used by this
	 * {@code PoseEstimationSubsystem}.
	 */
	private final SwerveDrivePoseEstimator m_poseEstimator;

	/**
	 * The timestamp of the latest {@code PhotonPipelineResult} received from the
	 * {@code PhotonCamera}.
	 */
	private double m_previousTimestamp = 0;

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
		m_camera = RobotBase.isSimulation() ? new PhotonCameraSimulator(cameraName, driveSubsystem, 5, 0.1)
				: new PhotonCamera(cameraName);
		this.m_driveSubsystem = driveSubsystem;
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
		PhotonPipelineResult pipelineResult = m_camera.getLatestResult();
		var timestamp = pipelineResult.getTimestampSeconds();
		if (timestamp != m_previousTimestamp && pipelineResult.hasTargets()) {
			m_previousTimestamp = timestamp;
			var target = pipelineResult.getBestTarget();
			var fiducialId = target.getFiducialId();
			Optional<Pose3d> tagPose = kFieldLayout == null ? Optional.empty()
					: kFieldLayout.getTagPose(fiducialId);
			if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
				Transform3d camToTarget = target.getBestCameraToTarget();
				Pose3d camPose = tagPose.get().transformBy(camToTarget.inverse());
				var visionMeasurement = camPose.transformBy(kRobotToCamera.inverse()).toPose2d();
				m_poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
				m_detectedPosePublisher.set(visionMeasurement);
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