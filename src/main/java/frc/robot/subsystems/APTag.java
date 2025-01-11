package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class APTag {
	PhotonCamera camera;

	public APTag() {
		camera = new PhotonCamera("Cool Camera");
	}

	public boolean getHasTargets() {
		PhotonPipelineResult result = camera.getAllUnreadResults().get(0);

		return result.hasTargets();
	}
}
