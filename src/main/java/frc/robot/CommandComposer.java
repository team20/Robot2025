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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

	private static Command scoreLevelInTeleop(double level, double clearanceHeight, Supplier<Command> levelCommand,
			double wristAngle) {
		return sequence(
				m_elevatorSubsystem.goToClearanceHeight(level, Units.inchesToMeters(clearanceHeight)),
				m_wristSubsystem.goToAngle(wristAngle),
				levelCommand.get());
	}

	public static Command scoreLevelOneInTeleop() {
		return scoreLevelInTeleop(kLevelOneHeight, 0.7, m_elevatorSubsystem::goToLevelOneHeight, kGrabberAngleLevelTwo)
				.withName("Score Level One in Teleop");
	}

	public static Command scoreLevelTwoInTeleop() {
		return scoreLevelInTeleop(kLevelTwoHeight, 0.7, m_elevatorSubsystem::goToLevelTwoHeight, kGrabberAngleLevelTwo)
				.withName("Score Level Two in Teleop");
	}

	public static Command removeAlgaeLevelThree() {
		return sequence(
				m_elevatorSubsystem.goToLevelTwoHeight(),
				m_wristSubsystem.goToAngle(kAlgaeWristHeight),
				m_elevatorSubsystem.goToAlgaeThreeHeight()).withName("Remove Algae Level Three");
	}

	public static Command removeAlgaeLevelTwo() {
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(kAlgaeWristHeight),
				m_elevatorSubsystem.goToAlgaeTwoHeight()).withName("Remove Algae Level Two");
	}

	public static Command releaseFlickAndDriveBack() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				parallel(
						m_wristSubsystem.goToAngle(kGrabberAngleLevelFour - 20),
						moveStraight(-0.3, 0.01, 1),
						m_cheeseStickSubsystem.grab())).withName("Release Flick And Drive Back");
	}

	public static Command scoreOptimized(Command align, Command pickup, int level) {
		switch (level) {
			case 4:
				return score(
						align, pickup, kLevelFourHeight, kGrabberAngleLevelFour,
						m_wristSubsystem.goToAngle(228)); // TODO: Change to 210
			case 3:
				return score(align, pickup, kLevelThreeHeight, kGrabberAngleLevelThree);
			case 2:
				return score(align, pickup, kLevelTwoHeight, kGrabberAngleLevelTwo);
			case 1:
				return score(align, pickup, kLevelOneHeight, kGrabberAngleLevelTwo);
		}
		return runOnce(() -> {
		});
	}

	private static Command score(Command align, Command pickup, double level, double wristAngle) {
		return score(align, pickup, level, wristAngle, runOnce(() -> {
		}));
	}

	private static Command score(Command align, Command pickup, double level, double wristAngle,
			Command followup) {
		return sequence(
				prepareToScore(align, pickup, level, wristAngle).withTimeout(3.5),
				score(.7, followup));// TODO: Optimize
	}

	static Command prepareToScore(Command align, double level, double wristAngle) {
		return prepareToScore(align, runOnce(() -> {
		}), level, wristAngle);
	}

	static Command prepareToScore(Command align, Command pickup, double level, double wristAngle) {
		return parallel(
				align,
				sequence(
						pickup,
						m_elevatorSubsystem.goToLevel(() -> level),
						m_wristSubsystem.goToAngle(wristAngle)));
	}

	public static Command score(double releaseDuration, Command followup) {
		return sequence(m_cheeseStickSubsystem.release(releaseDuration), followup);
	}

	public static Command score(Command align, int level) {
		return score(align, runOnce(() -> {
		}), level);
	}

	public static Command score(Command align, Command pickup, int level) {
		switch (level) {
			case 4:
				return score(
						align, pickup, kLevelFourHeight, kGrabberAngleLevelFour, kOffsets.get(level),
						m_wristSubsystem.goToAngle(200));
			case 3:
				return score(align, pickup, kLevelThreeHeight, kGrabberAngleLevelThree, kOffsets.get(level));
			case 2:
				return score(align, pickup, kLevelTwoHeight, kGrabberAngleLevelTwo, kOffsets.get(level));
			case 1:
				return score(align, pickup, kLevelOneHeight, kGrabberAngleLevelTwo, kOffsets.get(level));
		}
		return runOnce(() -> {
		});
	}

	private static Command score(Command align, Command pickup, double level, double wristAngle, double offset) {
		return score(align, pickup, level, wristAngle, offset, runOnce(() -> {
		}));
	}

	private static Command score(Command align, Command pickup, double level, double wristAngle, double offset,
			Command followup) {
		return sequence(
				prepareToScore(align, pickup, level, wristAngle),
				score(offset, 1.0, followup));
	}

	public static Command score(double offset, double releaseDuration, Command followup) {
		return sequence(
				moveStraight(offset, 0.01, 1), m_cheeseStickSubsystem.release(releaseDuration),
				parallel(followup, moveStraight(-2 * offset, 0.01, 1)));
	}

	public static Command toStation(int tagID) {
		return parallel(
				toTag(tagID, kRobotToStationTags),
				prepareForCoralPickup()).withName("Align to Station");
	}

	public static Command scoreOptimized(Command align, int level) {
		return scoreOptimized(align, runOnce(() -> {
		}), level).withName("Score optimized command of command");
	}

	public static Command prepareForCoralPickup() {
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(270)).withName("Prepare For Coral Pickup");
	}

	public static Command goToBase() {
		return sequence(
				m_wristSubsystem.goToAngle(270),
				m_elevatorSubsystem.goToBaseHeight()).withName("Go To Base");
	}

	public static Command pickupAtCoralStation() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_cheeseStickSubsystem.grab()).withName("Pick Up At Coral Station");
	}

	public static Command retractClimber() {
		return parallel(m_climberSubsystem.retract(), m_driveSubsystem.setNeutralMode(NeutralModeValue.Coast).asProxy())
				.finallyDo(() -> m_driveSubsystem.setDriveMotorNeutralMode(NeutralModeValue.Brake))
				.withName("Retract Climber and Drive Coast");
	}

	// TODO: AUTO SEQUENCES START HERE
	public static Command leave() {
		return m_driveSubsystem.driveCommand(() -> -0.25, () -> 0, () -> 0, () -> true).withTimeout(10)
				.withName("Leave Auto");
	}

	private static Command getOneScoreAndAlgae(Command align1, Command align2) {
		return sequence(
				scoreOptimized(align1, 4),
				align2.withTimeout(4),
				m_cheeseStickSubsystem.grab(),
				removeAlgaeLevelTwo(),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.01, 1)));
	}

	static Command getMiddleScoreAndAlgaeBlue() {
		return getOneScoreAndAlgae(toTag(21, kRobotToTagsLeft), toTag(21, kRobotToTags))
				.withName("Middle Score and Algae Blue");
	}

	static Command getMiddleScoreAndAlgaeRed() {
		return getOneScoreAndAlgae(toTag(10, kRobotToTagsLeft), toTag(10, kRobotToTags))
				.withName("Middle Score and Algae Red");
	}

	static Command getMiddleScoreAndAlgaePracticeField() {
		return getOneScoreAndAlgae(toTag(6, kRobotToTagsLeft), toTag(6, kRobotToTags))
				.withName("Middle Score and Algae Practice Field (6)");
	}

	static Command getLeftScoreAndAlgaeBlue() {
		return getOneScoreAndAlgae(toTag(20, kRobotToTagsLeft), toTag(20, kRobotToTags))
				.withName("Left Score and Algae Blue");
	}

	static Command getRightScoreAndAlgaeBlue() {
		return getOneScoreAndAlgae(toTag(22, kRobotToTagsLeft), toTag(22, kRobotToTags))
				.withName("Right Score and Algae Blue");
	}

	static Command getLeftScoreAndAlgaeRed() {
		return getOneScoreAndAlgae(toTag(11, kRobotToTagsLeft), toTag(11, kRobotToTags))
				.withName("Left Score and Algae Red");
	}

	static Command getRightScoreAndAlgaeRed() {
		return getOneScoreAndAlgae(toTag(9, kRobotToTagsLeft), toTag(9, kRobotToTags))
				.withName("Right Score and Algae Red");
	}

	public static Command getTwoScore(Command align1, int coralStationAlign, Command align2) {
		return sequence(
				scoreOptimized(align1, 4),
				m_cheeseStickSubsystem.grab(),
				toStation(coralStationAlign),
				waitSeconds(2),
				pickupAtCoralStation(),
				scoreOptimized(align2, 4),
				parallel(
						m_wristSubsystem.goToAngle(270),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getLeftTwoScoreBlue() {
		return getTwoScore(toTag(20, kRobotToTagsRight), 13, toTag(19, kRobotToTagsLeft))
				.withName("Blue-Left | Two Score ");
	}

	public static Command getRightTwoScoreBlue() {
		return getTwoScore(toTag(22, kRobotToTagsRight), 12, toTag(17, kRobotToTagsLeft))
				.withName("Blue-Right | Two Score ");
	}

	public static Command getLeftTwoScoreRed() {
		return getTwoScore(toTag(11, kRobotToTagsRight), 1, toTag(6, kRobotToTagsLeft))
				.withName("Red-Left | Two Score ");
	}

	public static Command getRightTwoScoreRed() {
		return getTwoScore(toTag(9, kRobotToTagsRight), 2, toTag(8, kRobotToTagsLeft))
				.withName("Red-Right | Two Score ");
	}

	public static Command getTwoScoreAndAlgae(Command align1, int coralStationAlign, Command align2, Command align3) {
		return sequence(
				scoreOptimized(align1, 4),
				m_cheeseStickSubsystem.grab(),
				toStation(coralStationAlign),
				waitSeconds(2),
				pickupAtCoralStation(),
				scoreOptimized(align2, 4),
				align3.withTimeout(4),
				m_cheeseStickSubsystem.grab(),
				removeAlgaeLevelTwo(),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getLeftTwoScoreAndAlgaeBlue() {
		return getTwoScoreAndAlgae(
				toTag(20, kRobotToTagsRight), 13, toTag(19, kRobotToTagsLeft), toTag(19, kRobotToTags))
						.withName("Blue-Left | Two Score and Algae ");
	}

	public static Command getRightTwoScoreAndAlgaeBlue() {
		return getTwoScoreAndAlgae(
				toTag(22, kRobotToTagsRight), 12, toTag(17, kRobotToTagsLeft), toTag(17, kRobotToTags))
						.withName("Blue-Right | Two Score and Algae ");
	}

	public static Command getLeftTwoScoreAndAlgaeRed() {
		return getTwoScoreAndAlgae(toTag(11, kRobotToTagsRight), 1, toTag(6, kRobotToTagsLeft), toTag(6, kRobotToTags))
				.withName("Red-Left | Two Score and Algae ");
	}

	public static Command getRightTwoScoreAndAlgaeRed() {
		return getTwoScoreAndAlgae(toTag(9, kRobotToTagsRight), 2, toTag(8, kRobotToTagsLeft), toTag(8, kRobotToTags))
				.withName("Red-Right | Two Score and Algae ");
	}

	public static Command getThreeScore(Command align1, int coralStationAlign, Command align2, Command align3) {
		return sequence(
				scoreOptimized(align1, 4),
				m_cheeseStickSubsystem.grab(),
				toStation(coralStationAlign),
				waitSeconds(2),
				pickupAtCoralStation(),
				scoreOptimized(align2, 4),
				m_cheeseStickSubsystem.grab(),
				toStation(coralStationAlign),
				waitSeconds(2),
				pickupAtCoralStation(),
				scoreOptimized(align3, 2),
				parallel(
						m_wristSubsystem.goToAngle(270),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getLeftThreeScoreBlue() {
		return getThreeScore(toTag(20, kRobotToTagsRight), 13, toTag(19, kRobotToTagsLeft), toTag(18, kRobotToTagsLeft))
				.withName("Blue-Left | Three Score ");
	}

	public static Command getRightThreeScoreBlue() {
		return getThreeScore(toTag(22, kRobotToTagsRight), 12, toTag(17, kRobotToTagsLeft), toTag(18, kRobotToTagsLeft))
				.withName("Blue-Right | Three Score ");
	}

	public static Command getLeftThreeScoreRed() {
		return getThreeScore(toTag(11, kRobotToTagsRight), 1, toTag(6, kRobotToTagsLeft), toTag(7, kRobotToTagsLeft))
				.withName("Red-Left | Three Score ");
	}

	public static Command getRightThreeScoreRed() {
		return getThreeScore(toTag(9, kRobotToTagsRight), 2, toTag(8, kRobotToTagsLeft), toTag(7, kRobotToTagsLeft))
				.withName("Red-Right | Three Score ");
	}

	public static Command testAbsoluteOrientation(double duration) {
		DoubleSupplier z = () -> 0;
		BooleanSupplier f = () -> false;
		return sequence(
				m_driveSubsystem.driveCommand(z, z, z, () -> 1, f).withTimeout(duration), // 90 degrees
				m_driveSubsystem.driveCommand(z, z, () -> -1, z, f).withTimeout(duration), // 180 degrees
				m_driveSubsystem.driveCommand(z, z, z, () -> -1, f).withTimeout(duration), // 270 degrees
				m_driveSubsystem.driveCommand(z, z, () -> 1, () -> 1, f).withTimeout(duration), // 45 degrees
				m_driveSubsystem.driveCommand(z, z, () -> 1, z, f).withTimeout(duration)); // 0 degrees
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