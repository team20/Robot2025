// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.AutoConstants;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
	private static VisionSystemSim visionSim;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	/**
	 * The {@code StructPublisher} for reporting the {@code Pose2d} of the
	 * robot in simulation.
	 */
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * Creates a new VisionIOPhotonVisionSim.
	 *
	 * @param name The name of the camera.
	 * @param poseSupplier Supplier for the robot pose to use in simulation.
	 */
	public VisionIOPhotonVisionSim(
			String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
		super(name, robotToCamera);

		this.poseSupplier = poseSupplier;

		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@Simulation", Pose2d.struct)
				.publish();

		// Initialize vision sim
		if (visionSim == null) {
			visionSim = new VisionSystemSim("Simulator");
			visionSim.addAprilTags(AutoConstants.kFieldLayout);
		}

		// Add sim camera
		var cameraProperties = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(camera, cameraProperties);
		visionSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		Pose2d pose = poseSupplier.get();
		m_posePublisher.set(pose);
		visionSim.update(pose);
		super.updateInputs(inputs);
	}
}