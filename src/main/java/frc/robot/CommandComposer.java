package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PathDriveCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CheeseStickSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static AlgaeGrabberSubsystem m_algaeGrabberSubsystem;
	private static CheeseStickSubsystem m_cheeseStickSubsystem;
	private static ClimberSubsystem m_climberSubsystem;
	private static ElevatorSubsystem m_elevatorSubsystem;
	private static WristSubsystem m_wristSubsystem;
	private static PoseEstimationSubsystem m_poseEstimationSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			AlgaeGrabberSubsystem algaeGrabberSubsystem,
			CheeseStickSubsystem cheeseStickSubsystem,
			ClimberSubsystem climberSubsystem,
			ElevatorSubsystem elevatorSubsystem,
			WristSubsystem wristSubsystem, PoseEstimationSubsystem poseEstimationSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_algaeGrabberSubsystem = algaeGrabberSubsystem;
		m_cheeseStickSubsystem = cheeseStickSubsystem;
		m_climberSubsystem = climberSubsystem;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_poseEstimationSubsystem = poseEstimationSubsystem;
	}

	public static Command get3ScoreNorth() {
		return new SelectCommand<Alliance>(Map
				.of(Alliance.Red, get3ScoreNorthRed(), Alliance.Blue, get3ScoreNorthBlue()),
				() -> DriverStation.getAlliance().get());
	}

	public static Command get3ScoreSouth() {
		return new SelectCommand<Alliance>(Map
				.of(Alliance.Red, get3ScoreSouthRed(), Alliance.Blue, get3ScoreSouthBlue()),
				() -> DriverStation.getAlliance().get());
	}

	private static Command get3ScoreNorthBlue() {
		return get3Score(toTag(20, kRobotToTagsRight), 13, toTag(19, kRobotToTagsRight), toTag(19, kRobotToTagsLeft));
	}

	private static Command get3ScoreNorthRed() {
		return get3Score(toTag(9, kRobotToTagsLeft), 2, toTag(8, kRobotToTagsLeft), toTag(8, kRobotToTagsRight));
	}

	private static Command get3ScoreSouthBlue() {
		return get3Score(toTag(22, kRobotToTagsLeft), 12, toTag(17, kRobotToTagsLeft), toTag(17, kRobotToTagsRight));
	}

	private static Command get3ScoreSouthRed() {
		return get3Score(toTag(11, kRobotToTagsRight), 1, toTag(6, kRobotToTagsRight), toTag(6, kRobotToTagsLeft));
	}

	private static Command get3Score(Command align1, int pickupTagID, Command align2, Command align3) {
		return sequence(
				score(align1, 4),
				toStation(pickupTagID),
				score(align2, 4, goToBase()),
				toStation(pickupTagID),
				score(align3, 4, goToBase()));
	}

	public static Command score(Command align, int level) {
		return score(align, level, runOnce(() -> {
		}));
	}

	public static Command score(Command align, int level, Command pickup) {
		switch (level) {
			case 4:
				return scoreWithAlignment(kLevelFourHeight, 0.015, kGrabberAngleLevelFour, align, pickup);
			case 3:
				return scoreWithAlignment(kLevelThreeHeight, 0.015, kGrabberAngleOthers, align, pickup);
			case 2:
				return scoreWithAlignment(kLevelTwoHeight, 0.05, kGrabberAngleOthers, align, pickup);
			case 1:
				return scoreWithAlignment(kLevelOneHeight, 0.55, kGrabberAngleOthers, align, pickup);
		}
		return runOnce(() -> {
		});
	}

	private static Command scoreWithAlignment(double level, double clearanceHeight, double wristAngle, Command align,
			Command pickup) {
		return sequence(
				parallel(
						sequence(
								pickup,
								m_elevatorSubsystem.goToClearanceHeight(level, clearanceHeight),
								m_wristSubsystem.goToAngle(wristAngle)),
						align),
				m_elevatorSubsystem.goToLevel(() -> level),
				score());
	}

	private static Command toStation(int tagID) {
		return sequence(
				parallel(
						m_wristSubsystem.goToAngle(270),
						toTag(tagID, kRobotToTags),
						m_elevatorSubsystem.goToCoralStationHeight()));
	}

	static Command scoreLevelInTeleop(double level, double clearanceHeight, Supplier<Command> levelCommand,
			double wristAngle) {
		return sequence(
				m_elevatorSubsystem.goToClearanceHeight(level, clearanceHeight),
				m_wristSubsystem.goToAngle(wristAngle),
				levelCommand.get());
	}

	public static Command score() {
		return sequence(m_cheeseStickSubsystem.release(), new WaitCommand(1), m_wristSubsystem.goToAngle(270));
	}

	public static Command scoreLevelOneInTeleop() {
		return scoreLevelInTeleop(kLevelOneHeight, 0.6, m_elevatorSubsystem::goToLevelOneHeight, kGrabberAngleOthers);
	}

	public static Command scoreLevelTwoInTeleop() {
		return scoreLevelInTeleop(kLevelTwoHeight, 0.1, m_elevatorSubsystem::goToLevelTwoHeight, kGrabberAngleOthers);
	}

	public static Command scoreLevelThreeInTeleop() {
		return scoreLevelInTeleop(
				kLevelThreeHeight, 0.15, m_elevatorSubsystem::goToLevelThreeHeight, kGrabberAngleOthers);
	}

	public static Command scoreLevelFourInTeleop() {
		return scoreLevelInTeleop(
				kLevelFourHeight, 0.15, m_elevatorSubsystem::goToLevelFourHeight, kGrabberAngleLevelFour);
	}

	public static Command removeAlgaeLevelThree() {
		return sequence(
				m_elevatorSubsystem.goToLevelTwoHeight(),
				m_wristSubsystem.goToAngle(kAlgaeWristHeight),
				m_elevatorSubsystem.goToAlgaeThreeHeight());
	}

	public static Command removeAlgaeLevelTwo() {
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(kAlgaeWristHeight),
				m_elevatorSubsystem.goToAlgaeTwoHeight());
	}

	public static Command releaseFlickAndDriveBack() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				parallel(
						m_wristSubsystem.goToAngle(kGrabberAngleLevelFour - 20),
						moveStraight(-0.3, 0.01, 1),
						m_cheeseStickSubsystem.grab()));
	}

	private static Command scoreLevelInAuto(Supplier<Command> levelCommand) {
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

	public static Command scoreLevelFourInAuto() {
		return scoreLevelInAuto(m_elevatorSubsystem::goToLevelFourHeight);
	}

	public static Command scoreLevelThreeInAuto() {
		return scoreLevelInAuto(m_elevatorSubsystem::goToLevelThreeHeight);
	}

	public static Command scoreLevelTwoInAuto() {
		return scoreLevelInAuto(m_elevatorSubsystem::goToLevelTwoHeight);
	}

	public static Command scoreLevelOneInAuto() {
		return scoreLevelInAuto(m_elevatorSubsystem::goToLevelOneHeight);
	}

	public static Command prepareForCoralPickup() {
		return parallel(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(270));
	}

	public static Command goToBase() {
		return sequence(
				m_wristSubsystem.goToAngle(270),
				m_elevatorSubsystem.goToBaseHeight());
	}

	public static Command pickupAtCoralStation() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_cheeseStickSubsystem.grab());
	}

	public static Command testAbsoluteOrientation(double duration) {
		DoubleSupplier z = () -> 0;
		BooleanSupplier f = () -> false;
		return sequence(
				m_driveSubsystem.driveCommand(z, z, z, () -> 1, z, f).withTimeout(duration), // 90 degrees
				m_driveSubsystem.driveCommand(z, z, () -> -1, z, z, f).withTimeout(duration), // 180 degrees
				m_driveSubsystem.driveCommand(z, z, z, () -> -1, z, f).withTimeout(duration), // 270 degrees
				m_driveSubsystem.driveCommand(z, z, () -> 1, () -> 1, z, f).withTimeout(duration), // 45 degrees
				m_driveSubsystem.driveCommand(z, z, () -> 1, z, z, f).withTimeout(duration)); // 0 degrees
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
				m_driveSubsystem.resetOdometry(Pose2d.kZero),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, Pose2d.kZero),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						new Pose2d(feetToMeters(distanceInFeet), 0, Rotation2d.kZero)),
				Commands.waitSeconds(2),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, Pose2d.kZero),
				Commands.waitSeconds(1),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, Pose2d.kZero));
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move
	 * the robot forward or backward.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param displacement the displacement (positive: forward, negative: backward)
	 *        of the movement
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 */
	public static Command moveStraight(double displacement, double distanceTolerance,
			double angleToleranceInDegrees) {
		return new DriveCommand(m_driveSubsystem, distanceTolerance, angleToleranceInDegrees, () -> {
			return m_driveSubsystem.getPose().plus(new Transform2d(displacement, 0, Rotation2d.kZero));
		});
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
				m_driveSubsystem.resetOdometry(Pose2d.kZero),
				new DriveCommand(m_driveSubsystem,
						distanceTolerance, angleTolerance, Pose2d.kZero),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						new Pose2d(sideLength, 0, Rotation2d.kCCW_90deg)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						new Pose2d(sideLength, sideLength, Rotation2d.k180deg)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						new Pose2d(0.0, sideLength, Rotation2d.kCW_90deg)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, Pose2d.kZero),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, Pose2d.kZero));
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
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1,
				0.05, 5,
				posesToClosestTag(3, robotToTags));
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	public static Command toTag(int tagID, Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1,
				0.16, 16,
				posesToTag(tagID, robotToTags));
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
	public static Command toTag(double distanceThresholdInMeters, double distanceTolerance,
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
	 * Creates a list of {@code Pose2d}s to automatically align the robot to the
	 * target {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a list of {@code Pose2d}s to automatically align the robot to the
	 *         target {@code AprilTag}
	 */
	public static List<Supplier<Pose2d>> posesToTag(int tagID,
			Transform2d... robotToTags) {
		return Arrays.stream(robotToTags).map(r -> (Supplier<Pose2d>) (() -> {
			Pose2d pose = pose(tagID);
			if (pose == null)
				return m_driveSubsystem.getPose();
			return m_poseEstimationSubsystem.odometryCentricPose(pose.plus(r));
		})).toList();
	}
}