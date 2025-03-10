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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
		return select(get3ScoreNorthRed(3), get3ScoreNorthBlue(3));
	}

	public static Command get3ScoreSouth() {
		return select(get3ScoreSouthRed(3), get3ScoreSouthBlue(3));
	}

	private static Command get3ScoreNorthBlue(int level) {
		return get3ScoreOptimized(
				toTag(20, kLevel2Offset.get(level), 0, kRobotToTagsRight),
				toTag(19, kLevel2Offset.get(level), 0, kRobotToTagsRight),
				toTag(19, kLevel2Offset.get(level), 0, kRobotToTagsLeft),
				13, 0.10, 0, kRobotToTagsRightReady);
	}

	private static Command get3ScoreNorthRed(int level) {
		return get3ScoreOptimized(
				toTag(9, kLevel2Offset.get(level), 0, kRobotToTagsLeft),
				toTag(8, kLevel2Offset.get(level), 0, kRobotToTagsLeft),
				toTag(8, kLevel2Offset.get(level), 0, kRobotToTagsRight),
				2, 0.10, 0, kRobotToTagsLeftReady);
	}

	private static Command get3ScoreSouthBlue(int level) {
		return get3ScoreOptimized(
				toTag(22, kLevel2Offset.get(level) + 0.09 + 0.01, -0.01, kRobotToTagsLeft),
				toTag(17, kLevel2Offset.get(level) + 0.09, 0.0, kRobotToTagsLeft),
				toTag(17, kLevel2Offset.get(level) + 0.09, 0.0, kRobotToTagsRight),
				12, 0.10, 0, kRobotToTagsLeftReady);
	}

	private static Command get3ScoreSouthRed(int level) {
		return get3ScoreOptimized(
				toTag(11, kLevel2Offset.get(level), 0, kRobotToTagsRight),
				toTag(6, kLevel2Offset.get(level), 0, kRobotToTagsRight),
				toTag(6, kLevel2Offset.get(level), 0, kRobotToTagsLeft),
				1, 0.10, 0, kRobotToTagsRightReady);
	}

	private static Command get3ScoreOptimized(Command align1, Command align2, Command align3, int stationTagID,
			double forward, double left, Transform2d... robotToTags) {
		return sequence(
				score(align1, goToBase(), 3), moveStraight(-0.5, 0.2, 20),
				toStation(stationTagID, forward, left, robotToTags), new WaitCommand(1),
				score(align2, goToBase(), 3), moveStraight(-0.5, 0.2, 20),
				toStation(stationTagID, forward, left, robotToTags), new WaitCommand(1),
				score(align3, goToBase(), 3));
	}

	public static Command scoreOptimized(Command align, int level) {
		return score(align, runOnce(() -> {
		}), level);
	}

	public static Command score(Command align, Command pickup, int level) {
		switch (level) {
			case 4:
				return score(
						align, pickup, kLevelFourHeight, kGrabberAngleLevelFour,
						m_wristSubsystem.goToAngle(210));
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

	private static Command toStation(int tagID, double forward, double left, Transform2d... robotToTags) {
		return parallel(
				toTag(tagID, forward, left, robotToTags), sequence(
						new WaitCommand(1),
						m_elevatorSubsystem.goToCoralStationHeight()));
	}

	static Command getMiddleScoreAndAlgaeBlue() {
		return getMiddleScoreAndAlgae(toTag(21, kRobotToTagsRight), toTag(21, kRobotToTags));
	}

	static Command getMiddleScoreAndAlgaeRed() {
		return getMiddleScoreAndAlgae(toTag(10, kRobotToTagsRight), toTag(10, kRobotToTags));
	}

	static Command scoreLevelInTeleop(double level, double clearanceHeight, Supplier<Command> levelCommand,
			double wristAngle) {
		return sequence(
				m_elevatorSubsystem.goToClearanceHeight(level, clearanceHeight),
				m_wristSubsystem.goToAngle(wristAngle),
				levelCommand.get());
	}

	public static Command scoreLevelOneInTeleop() {
		return scoreLevelInTeleop(kLevelOneHeight, 0.4, m_elevatorSubsystem::goToLevelOneHeight, kGrabberAngleOthers);
	}

	public static Command scoreLevelTwoInTeleop() {
		return scoreLevelInTeleop(kLevelTwoHeight, 0.4, m_elevatorSubsystem::goToLevelTwoHeight, kGrabberAngleOthers);
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

	private static Command getMiddleScoreAndAlgae(Command align1, Command align2) {
		return sequence(
				scoreOptimized(align1, 4),
				align2,
				m_cheeseStickSubsystem.grab(),
				removeAlgaeLevelTwo(),
				parallel(
						m_wristSubsystem.goToAngle(240),
						moveStraight(-0.7, 0.01, 1)));
	}

	public static Command prepareForCoralPickup() {
		return parallel(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(270));
	}

	public static Command goToBase() {
		return sequence(
				parallel(m_cheeseStickSubsystem.grab(), m_wristSubsystem.goToAngle(270)),
				m_elevatorSubsystem.goToBaseHeight());
	}

	public static Command pickupAtCoralStation() {
		return sequence(
				m_cheeseStickSubsystem.release(),
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_cheeseStickSubsystem.grab());
	}

	public static Command testLeftAlignment(int level, long distance, long duration, int... tagIDs) {
		return sequence(
				Arrays.stream(tagIDs)
						.mapToObj(
								t -> (Command) score(
										toTag(
												t,
												kLevel2Offset.get(level)
														+ kForwardAdjustment.getOrDefault(t, 0.0),
												kSideAdjustment.getOrDefault(t, 0.0),
												kRobotToTagsLeft),
										goToBase(), level))
						.map(c -> sequence(c, moveStraight(-distance, 0.01, 1), new WaitCommand(duration))).toList()
						.toArray(new Command[0]));
	}

	public static Command testRightAlignment(int level, long distance, long duration, int... tagIDs) {
		return sequence(
				Arrays.stream(tagIDs)
						.mapToObj(
								t -> (Command) score(
										toTag(
												t,
												kLevel2Offset.get(level)
														+ kForwardAdjustment.getOrDefault(t, 0.0),
												kSideAdjustment.getOrDefault(t, 0.0),
												kRobotToTagsRight),
										goToBase(), level))
						.map(c -> sequence(c, moveStraight(-distance, 0.01, 1), new WaitCommand(duration))).toList()
						.toArray(new Command[0]));
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

	public static Command toClosestTag(double forward, double left, Transform2d... robotToTags) {
		return toClosestTag(adjust(forward, left, robotToTags));
	}

	private static Transform2d[] adjust(double forward, double left, Transform2d... robotToTags) {
		return Arrays.stream(robotToTags)
				.map(t -> new Transform2d(t.getX() - forward, t.getY() - left, t.getRotation())).toList()
				.toArray(new Transform2d[0]);
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

	// /**
	// * Creates a {@code Command} to automatically align the robot to the target
	// * {@code AprilTag}.
	// *
	// * @param tagID the ID of the target {@code AprilTag}
	// * @param level the scoring level
	// * @param robotToTags the {@code Tranform2d} representing the pose of the
	// * target {@code AprilTag} relative to the robot when the robot is
	// * aligned
	// * @return a {@code Command} to automatically align the robot to the target
	// * {@code AprilTag}
	// */
	public static Command toTag(int tagID, double forward, double left, Transform2d... robotToTags) {
		return toTag(tagID, adjust(forward, left, robotToTags));
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
				0.01, 1, // TODO: Optimize
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

	private static Command select(Command commandRedAlliance, Command commandBlueAlliance) {
		return new SelectCommand<Object>(Map
				.of(Alliance.Red, commandRedAlliance, Alliance.Blue, commandBlueAlliance),
				() -> {
					Alliance alliance = DriverStation.getAlliance().get();
					var middle = kFieldLayout.getFieldLength() / 2;
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

	private static Alert alert(String text) {
		var a = new Alert(text, AlertType.kError);
		a.set(true);
		return a;
	}

}