package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PathDriveCommand;
import frc.robot.simulation.SimpleCheeseStickSubsystem;
import frc.robot.simulation.SimpleElevatorSubsystem;
import frc.robot.simulation.SimpleWristSubsystem;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static AlgaeGrabberSubsystem m_algaeGrabberSubsystem;
	private static SimpleCheeseStickSubsystem m_cheeseStickSubsystem;
	private static ClimberSubsystem m_climberSubsystem;
	private static SimpleElevatorSubsystem m_elevatorSubsystem;
	private static SimpleWristSubsystem m_wristSubsystem;
	private static PoseEstimationSubsystem m_poseEstimationSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			AlgaeGrabberSubsystem algaeGrabberSubsystem,
			SimpleCheeseStickSubsystem cheeseStickSubsystem,
			ClimberSubsystem climberSubsystem,
			SimpleElevatorSubsystem elevatorSubsystem,
			SimpleWristSubsystem wristSubsystem, PoseEstimationSubsystem poseEstimationSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_algaeGrabberSubsystem = algaeGrabberSubsystem;
		m_cheeseStickSubsystem = cheeseStickSubsystem;
		m_climberSubsystem = climberSubsystem;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_poseEstimationSubsystem = poseEstimationSubsystem;
	}

	private static Command scoreLevel(Supplier<Command> levelCommand) {
		return sequence(
				levelCommand.get(),
				m_wristSubsystem.goToAngle(35),
				m_elevatorSubsystem.lowerToScore(),
				m_cheeseStickSubsystem.release(),
				waitSeconds(0), // TODO: find the amount of time needed to retract
				levelCommand.get(),
				m_cheeseStickSubsystem.grab(),
				m_wristSubsystem.goToAngle(-90));
	}

	public static Command scoreLevelFour() {
		return scoreLevel(m_elevatorSubsystem::goToLevelFourHeight);
	}

	public static Command scoreLevelThree() {
		return scoreLevel(m_elevatorSubsystem::goToLevelThreeHeight);
	}

	public static Command scoreLevelTwo() {
		return scoreLevel(m_elevatorSubsystem::goToLevelTwoHeight);
	}

	public static Command scoreLevelOne() {
		return scoreLevel(m_elevatorSubsystem::goToLevelOneHeight);
	}

	public static Command scoreLevelFourWithLeftAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelFourHeight)
				.alongWith(toClosestTag(kRobotToTagsLeft));
	}

	public static Command scoreLevelThreeWithLeftAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelThreeHeight)
				.alongWith(toClosestTag(kRobotToTagsLeft));
	}

	public static Command scoreLevelTwoWithLeftAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelTwoHeight)
				.alongWith(toClosestTag(kRobotToTagsLeft));
	}

	public static Command scoreLevelOneWithLeftAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelOneHeight)
				.alongWith(toClosestTag(kRobotToTagsLeft));
	}

	public static Command scoreLevelFourWithRightAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelFourHeight)
				.alongWith(toClosestTag(kRobotToTagsRight));
	}

	public static Command scoreLevelThreeWithRightAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelThreeHeight)
				.alongWith(toClosestTag(kRobotToTagsRight));
	}

	public static Command scoreLevelTwoWithARightlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelTwoHeight)
				.alongWith(toClosestTag(kRobotToTagsRight));
	}

	public static Command scoreLevelOneWithRightAlignment() {
		return scoreLevel(m_elevatorSubsystem::goToLevelOneHeight)
				.alongWith(toClosestTag(kRobotToTagsRight));
	}

	public static Command prepareForCoralPickup() {
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(-90));
	}

	public static Command pickupAtCoralStation() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_cheeseStickSubsystem.grab());
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag}.
	 *
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag}
	 */
	public static Command toClosestTag(Transform2d... robotToTags) {
		return toClosestTag(3, 0.01, 1, 0.08, 8, robotToTags);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag}.
	 *
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag}
	 */
	public static Command toClosestTag(double distanceThresholdInMeters, double distanceTolerance,
			double angleToleranceInDegrees,
			double intermedateDistanceTolerance, double intermediateAngleToleranceInDegrees,
			Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleToleranceInDegrees,
				intermedateDistanceTolerance, intermediateAngleToleranceInDegrees,
				posesToClosestTag(distanceThresholdInMeters, robotToTags));
	}

	/**
	 * Creates a list of {@code Pose2d}s to automatically align the robot to the
	 * closest {@code AprilTag}.
	 *
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a list of {@code Pose2d}s to automatically align the robot to the
	 *         closest {@code AprilTag}
	 */
	public static List<Supplier<Pose2d>> posesToClosestTag(double distanceThresholdInMeters,
			Transform2d... robotToTags) {
		return Arrays.stream(robotToTags).map(r -> (Supplier<Pose2d>) (() -> {
			Pose2d closestTagPose = m_poseEstimationSubsystem.closestTagPose(180, distanceThresholdInMeters);
			if (closestTagPose == null)
				return m_driveSubsystem.getPose();
			return m_poseEstimationSubsystem.odometryCentricPose(closestTagPose.plus(r));
		})).toList();
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag} while driving the robot with joystick input
	 */
	public static Command driveWithLeftAlignment(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation) {
		return driveWithAlignment(forwardSpeed, strafeSpeed, rotation, kRobotToTagsLeft);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag} while driving the robot with joystick input
	 */
	public static Command driveWithRightAlignment(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation) {
		return driveWithAlignment(forwardSpeed, strafeSpeed, rotation, kRobotToTagsRight);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag} while driving the robot with joystick input
	 */
	public static Command driveWithAlignment(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation, Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1, 0.08, 8,
				posesToClosestTag(3, robotToTags)) {

			@Override
			public ChassisSpeeds chassisSpeeds() {
				ChassisSpeeds speeds = DriveSubsystem.chassisSpeeds(forwardSpeed, strafeSpeed, rotation);
				return speeds.plus(super.chassisSpeeds());
			}

		};
	}

	/**
	 * Returns a {@code Command} for moving forward and then backward.
	 * 
	 * @param distanceInFeet the distance in feet
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * 
	 * @return a {@code Command} for moving forward and then backward.
	 */
	public static Command moveForwardBackward(double distanceInFeet, double distanceTolerance,
			double angleTolerance) {
		return sequence(
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						pose(feetToMeters(distanceInFeet), 0, 0)),
				Commands.waitSeconds(2),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				Commands.waitSeconds(1),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)));
	}

	/**
	 * Returns a {@code Command} for moving the robot on a square.
	 * 
	 * @param sideLength the side length of the square in meters
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param timeout the maximum amount of the time given to the {@code Command}
	 * 
	 * @return a {@code Command} for moving the robot on a circle
	 */
	public static Command moveOnSquare(double sideLength, double distanceTolerance,
			double angleTolerance, double timeout) {
		return sequence(
				new DriveCommand(m_driveSubsystem,
						distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(sideLength, 0, 90)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						pose(sideLength, sideLength, 180)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, sideLength, 270)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0.0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)));
	}

	/**
	 * Returns a {@code Command} for aligning the robot to the specified
	 * {@code AprilTag}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param intermediateDistanceTolerance the distance error in meters which is
	 *        tolerable for intermeidate target {@code Pose2d}s
	 * @param intermediateAngleToleranceInDegrees the angle error in degrees which
	 *        is tolerable for intermeidate target {@code Pose2d}s
	 * @param robotToTagTransforms the {@code Pose2d}s of the {@code AprilTag}
	 *        relative to the center of the robot when the robit is aligned to the
	 *        ready and alignment poses
	 * @param robotToTagBackup the {@code Pose2d} of the {@code AprilTag} relative
	 *        to the center of the robot when the robit is aligned to the backup
	 *        pose
	 * @param tagIDs the IDs of the {@code AprilTag}s
	 * 
	 * @return a {@code Command} for aligning the robot to the specified
	 *         {@code AprilTag}s
	 */
	public static Command alignToTags(double distanceTolerance, double angleTolerance,
			double intermediateDistanceTolerance, double intermediateAngleToleranceInDegrees,
			List<Transform2d> robotToTagTransforms, Transform2d robotToTagBackup, int... tagIDs) {
		Pose2d previous = null;
		var commands = new LinkedList<Command>();
		for (int tagID : tagIDs) {
			var tagPose = kFieldLayout.getTagPose(tagID).get().toPose2d();
			var l = new LinkedList<Pose2d>();
			if (previous != null)
				l.add(previous);
			for (var r : robotToTagTransforms)
				l.add(tagPose.plus(r));
			previous = tagPose.plus(robotToTagBackup);
			var command = new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
					intermediateDistanceTolerance, intermediateAngleToleranceInDegrees, l.stream()
							.map(p -> (Supplier<Pose2d>) (() -> m_poseEstimationSubsystem.odometryCentricPose(p)))
							.toList());
			commands.add(command.andThen(new WaitCommand(.5)));
		}
		return sequence(commands.toArray(new Command[0]));
	}

}