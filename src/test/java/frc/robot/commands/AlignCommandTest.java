package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignCommandTest {

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
	}

	@AfterEach
	void shutdown() throws Exception {
	}

	@Test
	void testAngularDisplacement() {
		assertEquals(
				45.0,
				AlignCommand.angularDisplacement(
						new Pose2d(0, 0, Rotation2d.fromDegrees(45)), new Pose2d(0, 1, Rotation2d.fromDegrees(0)))
						.getDegrees(),
				0.1);
		assertEquals(
				135.0,
				AlignCommand.angularDisplacement(
						new Pose2d(0, 0, Rotation2d.fromDegrees(-45)), new Pose2d(0, 1, Rotation2d.fromDegrees(0)))
						.getDegrees(),
				0.1);
	}

}
