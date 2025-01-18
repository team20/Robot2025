package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.LinkedList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

public class DriveSubsystemTest {

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
	}

	@AfterEach
	void shutdown() throws Exception {
	}

	@Test
	void testForwardBackward() {
		var driveSubsystem = new DriveSubsystem();
		var l = new LinkedList<Double>();

		l.add(driveSubsystem.getPose().getX());
		assertEquals(0, l.getLast());

		driveSubsystem.drive(1, 0, 0, false);
		l.add(driveSubsystem.getPose().getX());
		// assertTrue(l.getLast() > 0); // TODO

		driveSubsystem.drive(-1, 0, 0, false);
		driveSubsystem.drive(-1, 0, 0, false);
		l.add(driveSubsystem.getPose().getX());
		// assertTrue(l.getLast() < 0); // TODO

		System.out.println("x-coordinate valus (meters): " + l);
		driveSubsystem.close();
	}

	@Test
	void testSideWays() {
		var driveSubsystem = new DriveSubsystem();
		var l = new LinkedList<Double>();

		l.add(driveSubsystem.getPose().getY());
		assertEquals(0, l.getLast());

		driveSubsystem.drive(0, 1, 0, false);
		l.add(driveSubsystem.getPose().getY());
		// assertTrue(l.getLast() > 0); // TODO

		driveSubsystem.drive(0, -1, 0, false);
		driveSubsystem.drive(0, -1, 0, false);
		l.add(driveSubsystem.getPose().getY());
		// assertTrue(l.getLast() < 0); // TODO

		System.out.println("y-coordinate values (meters): " + l);
		driveSubsystem.close();
	}

	@Test
	void testRotation() {
		var driveSubsystem = new DriveSubsystem();
		var l = new LinkedList<Double>();

		l.add(driveSubsystem.getPose().getRotation().getDegrees());
		assertEquals(0, l.getLast());

		driveSubsystem.drive(0, 0, 1, false);
		l.add(driveSubsystem.getPose().getRotation().getDegrees());
		// assertTrue(l.getLast() > 0); // TODO

		driveSubsystem.drive(0, 0, -1, false);
		driveSubsystem.drive(0, 0, -1, false);
		l.add(driveSubsystem.getPose().getRotation().getDegrees());
		// assertTrue(l.getLast() < 0); // TODO

		System.out.println("angles (degrees): " + l);
		driveSubsystem.close();
	}

}
