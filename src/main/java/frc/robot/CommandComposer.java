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

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
		return scoreLevelInTeleop(kLevelOneHeight, 0.7, m_elevatorSubsystem::goToLevelOneHeight, kGrabberAngleOthers)
				.withName("Score Level One in Teleop");
	}

	public static Command scoreLevelTwoInTeleop() {
		return scoreLevelInTeleop(kLevelTwoHeight, 0.7, m_elevatorSubsystem::goToLevelTwoHeight, kGrabberAngleOthers)
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

	public static Command score(Command align, int level) {
		return score(align, runOnce(() -> {
		}), level);
	}

	public static Command score(Command align, Command pickup, int level) {
		switch (level) {
			case 4:
				return score(
						align, pickup, kLevelFourHeight, kGrabberAngleLevelFour,
						m_wristSubsystem.goToAngle(228)); // TODO: Change to 210
			case 3:
				return score(align, pickup, kLevelThreeHeight, kGrabberAngleLevelThree);
			case 2:
				return score(align, pickup, kLevelTwoHeight, kGrabberAngleOthers);
			case 1:
				return score(align, pickup, kLevelOneHeight, kGrabberAngleOthers);
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
				prepareToScore(align, pickup, level, wristAngle),
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

	/**
	 * Returns a {@code Command} to score and remove one algae in the middle of the
	 * field.
	 * 
	 * @return a {@code Command} to score and remove one algae in the middle of the
	 *         field
	 */
	public static Command getMiddleScoreAndAlgae() {
		return select(
				getMiddleScoreAndAlgaeRed(),
				getMiddleScoreAndAlgaeBlue());
	}

	private static Command getMiddleScoreAndAlgae(Command align1, Command align2) {
		return sequence(
				score(align1, 4),
				align2.withTimeout(4),
				m_cheeseStickSubsystem.grab(),
				removeAlgaeLevelTwo(),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.01, 1)));
	}

	static Command getMiddleScoreAndAlgaeRed() {
		return getMiddleScoreAndAlgae(
				toTag(10, 4, 0.1, kRobotToTagsRight), // L4 scoring (<10cm errors at intermediate point)
				toTag(10, kForwrdAdjustmentAlgaeRemoval, 0.0, 0.1, kRobotToTags)) // algae removal (<10cm err at i. pnt)
						.withName("Middle Score and Algae Red");
	}

	static Command getMiddleScoreAndAlgaeBlue() {
		return getMiddleScoreAndAlgae(
				toTag(21, 4, 0.1, kRobotToTagsRight), // L4 scoring (<10cm errors at intermediate point)
				toTag(21, kForwrdAdjustmentAlgaeRemoval, 0.0, 0.1, kRobotToTags)) // algae removal (<10cm err at i. pnt)
						.withName("Middle Score and Algae Blue");
	}

	public static Command leave() {
		return m_driveSubsystem.driveCommand(() -> -0.25, () -> 0, () -> 0, () -> true).withTimeout(10)
				.withName("Leave Auto");
	}

	public static Command prepareForCoralPickup() {
		return parallel(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(270)).withName("Prepare For Coral Pickup");
	}

	public static Command goToBase() {
		return sequence(
				parallel(m_wristSubsystem.goToAngle(270), m_cheeseStickSubsystem.grab()),
				m_elevatorSubsystem.goToBaseHeight()).withName("Go To Base");
	}

	public static Command pickupAtCoralStation() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_cheeseStickSubsystem.grab()).withName("Pick Up At Coral Station");
	}

	public static Command testLeftAlignment(int level, double intermediateDistanceTolerance, double distance,
			double duration, int... tagIDs) {
		return sequence(
				Arrays.stream(tagIDs)
						.mapToObj(
								t -> (Command) score(
										toTag(
												t,
												kLevelOffset.get(level)
														+ kForwardAdjustment.getOrDefault(t, 0.0),
												kSideAdjustment.getOrDefault(t, 0.0), 0.1,
												kRobotToTagsLeft),
										goToBase(), level))
						.map(
								c -> sequence(
										c, moveStraight(-distance, intermediateDistanceTolerance, 1),
										new WaitCommand(duration)))
						.toList()
						.toArray(new Command[0]));
	}

	public static Command testRightAlignment(int level, double intermediateDistanceTolerance, double distance,
			double duration, int... tagIDs) {
		return sequence(
				Arrays.stream(tagIDs)
						.mapToObj(
								t -> (Command) score(
										toTag(
												t,
												kLevelOffset.get(level)
														+ kForwardAdjustment.getOrDefault(t, 0.0),
												kSideAdjustment.getOrDefault(t, 0.0), 0.1,
												kRobotToTagsRight),
										goToBase(), level))
						.map(
								c -> sequence(
										c, moveStraight(-distance, intermediateDistanceTolerance, 1),
										new WaitCommand(duration)))
						.toList()
						.toArray(new Command[0]));
	}

	public static Command retractClimber() {
		return parallel(m_climberSubsystem.retract(), m_driveSubsystem.setNeutralMode(NeutralModeValue.Coast).asProxy())
				.finallyDo(() -> m_driveSubsystem.setDriveMotorNeutralMode(NeutralModeValue.Brake))
				.withName("Retract Climber and Drive Coast");
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
				0.05, 5, // TODO: Optimize
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
		return toTag(tagID, 0.16, robotToTags);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the scoring level
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	public static Command toTag(int tagID, int level, double intermediateDistanceTolerance,
			Transform2d... robotToTags) {
		return toTag(
				tagID, kLevelOffset.get(level) + kForwardAdjustment.getOrDefault(tagID, 0.0),
				kSideAdjustment.getOrDefault(tagID, 0.0), intermediateDistanceTolerance, robotToTags);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param sideAdjustment the additional distance to strafe left/right
	 *        (positive: left when facing toward the tag)
	 * @param intermediateDistanceTolerance the distance error in meters which is
	 *        tolerable for intermeidate target {@code Pose2d}s
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	public static Command toTag(int tagID, double forwardAdjustment, double sideAdjustment,
			double intermediateDistanceTolerance,
			Transform2d... robotToTags) {
		return toTag(tagID, intermediateDistanceTolerance, adjust(forwardAdjustment, sideAdjustment, robotToTags));
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @param intermediateDistanceTolerance the distance error in meters which is
	 *        tolerable for intermeidate target {@code Pose2d}s
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	public static Command toTag(int tagID, double intermediateDistanceTolerance, Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1,
				intermediateDistanceTolerance, 16,
				posesToTag(tagID, robotToTags));
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

	public static Command get3ScoreNorth(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return select(
				get3ScoreNorthRed(level, distance, waitTime, intermediateDistanceTolerance),
				get3ScoreNorthBlue(level, distance, waitTime, intermediateDistanceTolerance));
	}

	public static Command get3ScoreSouth(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return select(
				get3ScoreSouthRed(level, distance, waitTime, intermediateDistanceTolerance),
				get3ScoreSouthBlue(level, distance, waitTime, intermediateDistanceTolerance));
	}

	private static Command get3ScoreNorthBlue(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return get3Score(
				toTag(20, level, intermediateDistanceTolerance, kRobotToTagsRight),
				toTag(19, level, intermediateDistanceTolerance, kRobotToTagsRight),
				toTag(19, level, intermediateDistanceTolerance, kRobotToTagsLeft), level, distance,
				13, waitTime, intermediateDistanceTolerance, kRobotToTagsRightReady);
	}

	private static Command get3ScoreNorthRed(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return get3Score(
				toTag(9, level, intermediateDistanceTolerance, kRobotToTagsLeft),
				toTag(8, level, intermediateDistanceTolerance, kRobotToTagsLeft),
				toTag(8, level, intermediateDistanceTolerance, kRobotToTagsRight), level, distance,
				2, waitTime, intermediateDistanceTolerance, kRobotToTagsLeftReady);
	}

	private static Command get3ScoreSouthBlue(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return get3Score(
				toTag(22, level, intermediateDistanceTolerance, kRobotToTagsLeft),
				toTag(17, level, intermediateDistanceTolerance, kRobotToTagsLeft),
				toTag(17, level, intermediateDistanceTolerance, kRobotToTagsRight), level, distance,
				12, waitTime, intermediateDistanceTolerance, kRobotToTagsLeftReady);
	}

	private static Command get3ScoreSouthRed(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return get3Score(
				toTag(11, level, intermediateDistanceTolerance, kRobotToTagsRight),
				toTag(6, level, intermediateDistanceTolerance, kRobotToTagsRight),
				toTag(6, level, intermediateDistanceTolerance, kRobotToTagsLeft), level, distance,
				1, waitTime, intermediateDistanceTolerance, kRobotToTagsRightReady);
	}

	private static Command get3Score(Command align1, Command align2, Command align3, int level, double distance,
			int stationTagID, double waitTime,
			double intermediateDistanceTolerance, Transform2d... robotToTags) {
		return sequence(
				score(align1, goToBase(), level), moveStraight(-distance, intermediateDistanceTolerance, 20),
				toStation(stationTagID, kForwrdAdjustmentCoralStation, intermediateDistanceTolerance, robotToTags),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(align2, goToBase(), level), moveStraight(-distance, intermediateDistanceTolerance, 20),
				toStation(stationTagID, kForwrdAdjustmentCoralStation, intermediateDistanceTolerance, robotToTags),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(align3, goToBase(), level));
	}

	private static Command toStation(int tagID, double forwardAdjustment, double intermediateDistanceTolerance,
			Transform2d... robotToTags) {
		return parallel(
				toTag(tagID, forwardAdjustment, 0, intermediateDistanceTolerance, robotToTags),
				m_elevatorSubsystem.goToCoralStationHeight());
	}

	public static Command toClosestTag(double forwardAdjustment, double sideAdjustment, Transform2d... robotToTags) {
		return toClosestTag(adjust(forwardAdjustment, sideAdjustment, robotToTags));
	}

	private static Transform2d[] adjust(double forwardAdjustment, double sideAdjustment, Transform2d... robotToTags) {
		return Arrays.stream(robotToTags)
				.map(t -> new Transform2d(t.getX() - forwardAdjustment, t.getY() - sideAdjustment, t.getRotation()))
				.toList()
				.toArray(new Transform2d[0]);
	}

	private static Command select(Command commandRedAlliance, Command commandBlueAlliance, boolean safetyStop) {
		return new SelectCommand<Object>(Map
				.of(Alliance.Red, commandRedAlliance, Alliance.Blue, commandBlueAlliance),
				() -> {
					Alliance alliance = DriverStation.getAlliance().get();
					var middle = kFieldLayout.getFieldLength() / 2;
					if (safetyStop)
						try {
							var confidence = m_poseEstimationSubsystem.confidence();
							if (confidence < 0.3)
								return alert("Pose Confidence (" + confidence + ") Too Low!");
							var x = m_poseEstimationSubsystem.getEstimatedPose().getX();
							if ((alliance == DriverStation.Alliance.Blue && x > middle)
									|| (alliance == DriverStation.Alliance.Red && x < middle)) {
								return alert("Strange Robot Position (" + alliance + " Alliance)!");
							}
						} catch (Exception e) {
						}
					return alliance;
				});
	}

	private static Command select(Command commandRedAlliance, Command commandBlueAlliance) {
		return select(commandRedAlliance, commandBlueAlliance, true);
	}

	private static Alert alert(String text) {
		var a = new Alert(text, AlertType.kError);
		a.set(true);
		return a;
	}

	public static Command getScoringTestNorth(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return select(
				getScoringTestNorthRed(level, distance, waitTime, intermediateDistanceTolerance),
				getScoringTestNorthBlue(level, distance, waitTime, intermediateDistanceTolerance), false);
	}

	public static Command getScoringTestSouth(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return select(
				getScoringTestSouthRed(level, distance, waitTime, intermediateDistanceTolerance),
				getScoringTestSouthBlue(level, distance, waitTime, intermediateDistanceTolerance), false);
	}

	private static Command getScoringTestNorthRed(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return getScoringTest(2, level, distance, waitTime, intermediateDistanceTolerance, 8);
	}

	private static Command getScoringTestNorthBlue(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return getScoringTest(13, level, distance, waitTime, intermediateDistanceTolerance, 19);
	}

	private static Command getScoringTestSouthRed(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return getScoringTest(1, level, distance, waitTime, intermediateDistanceTolerance, 6);
	}

	private static Command getScoringTestSouthBlue(int level, double distance, double waitTime,
			double intermediateDistanceTolerance) {
		return getScoringTest(12, level, distance, waitTime, intermediateDistanceTolerance, 17);
	}

	private static Command getScoringTest(int stationTagID, int level, double distance, double waitTime,
			double intermediateDistanceTolerance, int... tagIDs) {
		SequentialCommandGroup c = new SequentialCommandGroup();
		for (var tagID : tagIDs)
			c.addCommands(
					sequence(
							toStation(
									stationTagID, kForwrdAdjustmentCoralStation, intermediateDistanceTolerance,
									kRobotToTags),
							parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
							score(toTag(tagID, kRobotToTagsLeft), goToBase(), level),
							moveStraight(-distance, intermediateDistanceTolerance, 20),
							toStation(
									stationTagID, kForwrdAdjustmentCoralStation, intermediateDistanceTolerance,
									kRobotToTags),
							parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
							score(toTag(tagID, kRobotToTagsRight), goToBase(), level),
							moveStraight(-distance, intermediateDistanceTolerance, 20)));
		return c;
	}

	public static Command forwardBackwardSpeedTest(int iterations, double displacement, double distanceTolerance,
			double angleToleranceInDegrees) {
		SequentialCommandGroup g = new SequentialCommandGroup();
		for (int i = 0; i < iterations; i++)
			g.addCommands(
					CommandComposer.moveStraight(displacement, distanceTolerance, angleToleranceInDegrees),
					new WaitCommand(1.0),
					CommandComposer.moveStraight(-displacement, distanceTolerance, angleToleranceInDegrees),
					new WaitCommand(1.0));
		return g;
	}

}