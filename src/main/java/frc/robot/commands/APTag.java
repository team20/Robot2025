//TODO: Make this into a subsystem inline

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class APTag {
	PhotonCamera camera;

	public APTag() {
		camera = new PhotonCamera("Cool camera");
	}

	public boolean getHasTargets() {
		List<PhotonPipelineResult> results = camera.getAllUnreadResults();

		if (results.size() < 1) {
			return false;
		}

		PhotonPipelineResult result = camera.getAllUnreadResults().get(0);

		SmartDashboard.putBoolean("Has Targets", result.hasTargets());

		if (result.hasTargets()) {
			return result.getBestTarget().getFiducialId() == 1;
		}

		else {
			return false;
		}
	}
}