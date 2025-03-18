package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
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
	static AlgaeGrabberSubsystem m_algaeGrabberSubsystem;
	private static CheeseStickSubsystem m_cheeseStickSubsystem;
	static ClimberSubsystem m_climberSubsystem;
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
		return toTag(() -> m_poseEstimationSubsystem.closestTagID(180, 3), 0, robotToTags);
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score.
	 * 
	 * @param level the target elevator level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral from the pocket
	 * @return a {@code Command} to prepare the robot to score
	 */
	public static Command prepareToScore(int level, boolean pickup) {
		var c = pickup ? new SequentialCommandGroup(goToBase()) : new SequentialCommandGroup();
		c.addCommands(
				m_elevatorSubsystem.goToLevel(() -> kLevelElevatorHeights.get(level)),
				m_wristSubsystem.goToAngle(kLevelWristAngles.get(level)));
		return c;
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target elevator level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral from the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to prepare the robot to score
	 */
	public static Command prepareToScore(int tagID, int level, boolean pickup, Transform2d... robotToTags) {
		return prepareToScore(() -> tagID, level, pickup, robotToTags);
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score at the closest
	 * {@code AprilTag}.
	 * 
	 * @param level the target elevator level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral from the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to prepare the robot to score at the closest
	 *         {@code AprilTag}
	 */
	public static Command prepareToScoreClosest(int level, boolean pickup, Transform2d... robotToTags) {
		return prepareToScore(() -> m_poseEstimationSubsystem.closestTagID(180, 3), level, pickup, robotToTags);
	}

	/**
	 * Returns a {@code Command} to score.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target elevator level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral from the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to score
	 */
	public static Command score(int tagID, int level, boolean pickup, Transform2d... robotToTags) {
		return score(prepareToScore(tagID, level, pickup, robotToTags), level);
	}

	/**
	 * Returns a {@code Command} to score at the closest {@code AprilTag}.
	 * 
	 * @param level the target elevator level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral from the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to score at the closest {@code AprilTag}
	 */
	public static Command scoreClosest(int level, boolean pickup, Transform2d... robotToTags) {
		return score(prepareToScoreClosest(level, pickup, robotToTags), level);
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

	private static Command getMiddleScoreAndAlgae(Command score, Command align2) {
		return sequence(
				score,
				align2.withTimeout(4),
				m_cheeseStickSubsystem.grab(),
				removeAlgaeLevelTwo(),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.01, 1)));
	}

	private static Command getMiddleScoreAndAlgaeRed() {
		return getMiddleScoreAndAlgae(
				score(10, 4, true, kRobotToTagsRight),
				toTag(10, kForwrdAdjustmentAlgaeRemoval, kRobotToTags))
						.withName("Middle Score and Algae Red");
	}

	private static Command getMiddleScoreAndAlgaeBlue() {
		return getMiddleScoreAndAlgae(
				score(21, 4, true, kRobotToTagsRight),
				toTag(21, kForwrdAdjustmentAlgaeRemoval, kRobotToTags))
						.withName("Middle Score and Algae Blue");
	}

	static Command getMiddleScoreAndAlgaePracticeField() {
		return getMiddleScoreAndAlgae(toTag(6, kRobotToTagsLeft), toTag(6, kRobotToTags))
				.withName("Middle Score and Algae Practice Field (6)");
	}

	public static Command leave() {
		return m_driveSubsystem.driveCommand(() -> -0.25, () -> 0, () -> 0, () -> true).withTimeout(10)
				.withName("Leave Auto");
	}

	public static Command getTwoScore(Command align1, int coralStationAlign, Command align2) {
		return sequence(
				score(align1, 4),
				m_cheeseStickSubsystem.grab(),
				toStation(coralStationAlign),
				waitSeconds(2),
				pickupAtCoralStation(),
				score(align2, kGrabberAngleLevelFour),
				parallel(
						m_wristSubsystem.goToAngle(270),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getTwoScoreRedLeftSide() {
		return getTwoScore(toTag(11, kRobotToTagsRight), 1, toTag(6, kRobotToTagsLeft))
				.withName("Red-Left | Two Score ");
	}

	public static Command toStation(int tagID) {
		return parallel(
				toTag(tagID, kRobotToStationTags),
				prepareForCoralPickup()).withName("Align to Station");

	}

	public static Command prepareForCoralPickup() {
		return sequence(
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

	/**
	 * Returns a {@code Command} to perform a 3-score auto in the north of the
	 * field.
	 * 
	 * @param distance the distance to retreat after scoring
	 * @param waitTime the time to load a coral at the coral station
	 * @return a {@code Command} to perform a 3-score auto in the north of the
	 *         field
	 */
	public static Command get3ScoreNorth(double distance, double waitTime) {
		return select(
				get3ScoreNorthRed(distance, waitTime),
				get3ScoreNorthBlue(distance, waitTime));
	}

	/**
	 * Returns a {@code Command} to perform a 3-score auto in the south of the
	 * field.
	 * 
	 * @param distance the distance to retreat after scoring
	 * @param waitTime the time to load a coral at the coral station
	 * @return a {@code Command} to perform a 3-score auto in the south of the
	 *         field
	 */
	public static Command get3ScoreSouth(double distance, double waitTime) {
		return select(
				get3ScoreSouthRed(distance, waitTime),
				get3ScoreSouthBlue(distance, waitTime));
	}

	/**
	 * Returns a {@code Command} to perform a 3-score auto in the north of the
	 * field as a memeber of the blue alliance.
	 * 
	 * @param distance the distance to retreat after scoring
	 * @param waitTime the time to load a coral at the coral station
	 * @return a {@code Command} to perform a 3-score auto iin the north of the
	 *         field as a memeber of the blue alliance
	 */
	private static Command get3ScoreNorthBlue(double distance, double waitTime) {
		return sequence(
				score(20, 4, true, kRobotToTagsRight),
				toStation(13, kForwrdAdjustmentCoralStation, kRobotToTagsRightReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(19, 4, true, kRobotToTagsRight),
				toStation(13, kForwrdAdjustmentCoralStation, kRobotToTagsRightReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(19, 4, true, kRobotToTagsLeft));
	}

	/**
	 * Returns a {@code Command} to perform a 3-score auto in the north of the
	 * field as a memeber of the red alliance.
	 * 
	 * @param distance the distance to retreat after scoring
	 * @param waitTime the time to load a coral at the coral station
	 * @return a {@code Command} to perform a 3-score auto iin the north of the
	 *         field as a memeber of the red alliance
	 */
	private static Command get3ScoreNorthRed(double distance, double waitTime) {
		return sequence(
				score(9, 4, true, kRobotToTagsLeft),
				toStation(2, kForwrdAdjustmentCoralStation, kRobotToTagsLeftReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(8, 4, true, kRobotToTagsLeft),
				toStation(2, kForwrdAdjustmentCoralStation, kRobotToTagsLeftReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(8, 4, true, kRobotToTagsRight));
	}

	/**
	 * Returns a {@code Command} to perform a 3-score auto in the south of the
	 * field as a memeber of the blue alliance.
	 * 
	 * @param distance the distance to retreat after scoring
	 * @param waitTime the time to load a coral at the coral station
	 * @return a {@code Command} to perform a 3-score auto iin the south of the
	 *         field as a memeber of the blue alliance
	 */
	private static Command get3ScoreSouthBlue(double distance, double waitTime) {
		return sequence(
				score(22, 4, true, kRobotToTagsLeft),
				toStation(12, kForwrdAdjustmentCoralStation, kRobotToTagsLeftReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(17, 4, true, kRobotToTagsLeft),
				toStation(12, kForwrdAdjustmentCoralStation, kRobotToTagsLeftReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(17, 4, true, kRobotToTagsRight));
	}

	/**
	 * Returns a {@code Command} to perform a 3-score auto in the south of the
	 * field as a memeber of the red alliance.
	 * 
	 * @param distance the distance to retreat after scoring
	 * @param waitTime the time to load a coral at the coral station
	 * @return a {@code Command} to perform a 3-score auto iin the south of the
	 *         field as a memeber of the red alliance
	 */
	private static Command get3ScoreSouthRed(double distance, double waitTime) {
		return sequence(
				score(11, 4, true, kRobotToTagsRight),
				toStation(1, kForwrdAdjustmentCoralStation, kRobotToTagsRightReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(6, 4, true, kRobotToTagsRight),
				toStation(1, kForwrdAdjustmentCoralStation, kRobotToTagsRightReady),
				parallel(m_wristSubsystem.goToAngle(270), new WaitCommand(waitTime)),
				score(6, 4, true, kRobotToTagsLeft));
	}

	/**
	 * Returns a {@code Command} to align to the specified coral station.
	 * 
	 * @param tagID the ID of the {@code AprilTag} attached to the coral station
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to align to the specified coral station
	 */
	private static Command toStation(int tagID, double forwardAdjustment, Transform2d... robotToTags) {
		return parallel(
				toTag(tagID, forwardAdjustment, robotToTags),
				m_elevatorSubsystem.goToCoralStationHeight());
	}

	/**
	 * Returns a {@code Command} to score at the specified level.
	 * 
	 * @param prepare a {@code Command} that prepares the robot to score
	 * @param level the scoring level
	 * @return a {@code Command} to score at the specified level
	 */
	private static Command score(Command prepare, int level) {
		var c = sequence(prepare, m_cheeseStickSubsystem.release(0.7));
		return level == 4 ? c.andThen(m_wristSubsystem.goToAngle(kLevelWristAngles.get(level) + 10)) : c;
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target elevator level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral from the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to prepare the robot to score
	 */
	private static Command prepareToScore(Supplier<Integer> tagID, int level, boolean pickup,
			Transform2d... robotToTags) {
		return parallel(
				toTag(tagID, kLevelForwardAdjustments.getOrDefault(level, 0.0), robotToTags),
				prepareToScore(level, pickup));
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	private static Command toTag(int tagID, double forwardAdjustment, Transform2d... robotToTags) {
		return toTag(() -> tagID, forwardAdjustment, robotToTags);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the target
	 * {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the target
	 *         {@code AprilTag}
	 */
	private static Command toTag(Supplier<Integer> tagID, double forwardAdjustment, Transform2d... robotToTags) {
		return new PathDriveCommand(m_driveSubsystem, 0.01, 1,
				0.16, 16, // TODO: Optimize
				posesToTag(tagID, forwardAdjustment, robotToTags));
	}

	/**
	 * Creates a list of {@code Pose2d}s to automatically align the robot to the
	 * target {@code AprilTag}.
	 *
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a list of {@code Pose2d}s to automatically align the robot to the
	 *         target {@code AprilTag}
	 */
	private static List<Supplier<Pose2d>> posesToTag(Supplier<Integer> tagID, double forwardAdjustment,
			Transform2d... robotToTags) {
		return Arrays.stream(robotToTags).map(r -> (Supplier<Pose2d>) (() -> {
			var tID = tagID.get();
			if (tID == null)
				return m_driveSubsystem.getPose();
			Pose2d pose = pose(tID);
			if (pose == null)
				return m_driveSubsystem.getPose();
			var t = adjust(
					forwardAdjustment + kTagForwardAdjustments.getOrDefault(tID, 0.0),
					kTagSideAdjustments.getOrDefault(tID, 0.0), r);
			return m_poseEstimationSubsystem.odometryCentricPose(
					pose.plus(t));
		})).toList();
	}

	/**
	 * Applies the specified adjustments to the specified {@code Transform2d}.
	 * 
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param sideAdjustment the additional distance to move to left/right
	 *        (positive: strafe left when facing toward the tag)
	 * @param t a {@code Transform2d}
	 * @return the resulting {@code Transform2d}
	 */
	private static Transform2d adjust(double forwardAdjustment, double sideAdjustment, Transform2d t) {
		return new Transform2d(t.getX() - forwardAdjustment, t.getY() - sideAdjustment, t.getRotation());
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
	private static Command toTag(int tagID, Transform2d... robotToTags) {
		return toTag(tagID, 0.16, robotToTags);
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