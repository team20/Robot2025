// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
	private final VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIOInputs[] inputs;

	private final Alert[] disconnectedAlerts;

	public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
		this.consumer = consumer;/*  */
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputs[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputs();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
					AlertType.kWarning);
		}
	}

	@Override
	public void periodic() {
		// if (Constants.currentMode == Mode.SIM && !Constants.visionSim) {
		// consumer.accept(
		// poseSupplier.get(), Timer.getFPGATimestamp(),
		// VecBuilder.fill(0.1, 0.1, 0.1));
		// return;
		// }

		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = AutoConstants.kFieldLayout.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
						|| (observation.tagCount() == 1
								&& observation.ambiguity() > AutoConstants.maxAmbiguity) // Cannot be high ambiguity
						|| Math.abs(observation.pose().getZ()) > AutoConstants.maxZError // Must have realistic Z
																							// coordinate

						// Must be within the field boundaries
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX() > AutoConstants.kFieldLayout.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > AutoConstants.kFieldLayout.getFieldWidth();

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = AutoConstants.linearStdDevBaseline * stdDevFactor;
				double angularStdDev = AutoConstants.angularStdDevBaseline * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= AutoConstants.linearStdDevMegatag2Factor;
					angularStdDev *= AutoConstants.angularStdDevMegatag2Factor;
				}
				if (cameraIndex < AutoConstants.cameraStdDevFactors.length) {
					linearStdDev *= AutoConstants.cameraStdDevFactors[cameraIndex];
					angularStdDev *= AutoConstants.cameraStdDevFactors[cameraIndex];
				}

				// Send vision observation
				consumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

	}

	@FunctionalInterface
	public interface VisionConsumer {
		void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
	}

}