package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

	/**
	 * Returns a {@code Command} to align to the specified {@code AprilTag} and then
	 * remove the algae at level two.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @return a {@code Command} to align to the specified {@code AprilTag} and then
	 *         remove the algae at level two
	 */
	public static Command removeAlgaeLevelTwo(int tagID) {
		return removeAlgaeLevelTwo(() -> tagID);
	}

	/**
	 * Returns a {@code Command} to align to the specified {@code AprilTag} and then
	 * remove the algae at level two.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @return a {@code Command} to align to the specified {@code AprilTag} and then
	 *         remove the algae at level two
	 */
	private static Command removeAlgaeLevelTwo(Supplier<Integer> tagID) {
		return sequence(
				toTag(tagID, 0, kRobotToTags), // .withTimeout(4), This timeout seems to affect alignment accuracy
				m_cheeseStickSubsystem.grab(),
				removeAlgaeLevelTwo());
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
	 * Returns a {@code Command} to prepare the robot to score at the specified
	 * level.
	 * 
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @return a {@code Command} to prepare the robot to score at the specified
	 *         level
	 */
	public static Command prepareToScore(int level, boolean pickup) {
		switch (level) {
			case 4:
				return prepareToScore(kLevelFourHeight, kGrabberAngleLevelFour, pickup);
			case 3:
				return prepareToScore(kLevelThreeHeight, kGrabberAngleLevelThree, pickup);
			case 2:
				return prepareToScore(kLevelTwoHeight, kGrabberAngleLevelTwo, pickup);
		}
		return runOnce(() -> {
		});
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score based on the
	 * specified elevator level and wrist angle.
	 * 
	 * @param elevatorLevel the target elevator level
	 * @param wristAngle the target wrist angle
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @return {@code Command} to prepare the robot to score based on the
	 *         specified elevator level and wrist angle
	 */
	public static Command prepareToScore(double elevatorLevel, double wristAngle, boolean pickup) {
		var c = pickup ? new SequentialCommandGroup(goToBase()) : new SequentialCommandGroup();
		c.addCommands(
				m_elevatorSubsystem.goToLevel(() -> elevatorLevel),
				m_wristSubsystem.goToAngle(wristAngle));
		return c;
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score at the specified
	 * {@code AprilTag} and level.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to prepare the robot to score at the specified
	 *         {@code AprilTag} and level
	 */
	public static Command prepareToScore(int tagID, int level, boolean pickup, Transform2d... robotToTags) {
		return prepareToScore(() -> tagID, level, pickup, robotToTags);
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score at the closest
	 * {@code AprilTag} and the specified level.
	 * 
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to prepare the robot to score at the closest
	 *         {@code AprilTag} and the specified level
	 */
	public static Command prepareToScoreClosest(int level, boolean pickup, Transform2d... robotToTags) {
		return prepareToScore(() -> m_poseEstimationSubsystem.closestTagID(), level, pickup, robotToTags);
	}

	/**
	 * Returns a {@code Command} to prepare the robot to score at the specified
	 * {@code AprilTag} and level.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to prepare the robot to score at the specified
	 *         {@code AprilTag} and level
	 */
	private static Command prepareToScore(Supplier<Integer> tagID, int level, boolean pickup,
			Transform2d... robotToTags) {
		return parallel(
				toTag(tagID, kLevelForwardOffsets.getOrDefault(level, 0.0), robotToTags),
				prepareToScore(level, pickup));
	}

	/**
	 * Returns a {@code Command} to score at the specified {@code AprilTag} and
	 * level.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param retreatDistance the retreat distance at the end of scoring
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to score at the specified {@code AprilTag} and
	 *         level
	 */
	public static Command score(int tagID, int level, boolean pickup, double retreatDistance,
			Transform2d... robotToTags) {
		return score(() -> tagID, level, pickup, retreatDistance, robotToTags);
	}

	/**
	 * Returns a {@code Command} to score at the specified {@code AprilTag} and
	 * level.
	 * 
	 * @param tagID the ID of the target {@code AprilTag}
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param retreatDistance the retreat distance at the end of scoring
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to score at the specified {@code AprilTag} and
	 *         level
	 */
	private static Command score(Supplier<Integer> tagID, int level, boolean pickup, double retreatDistance,
			Transform2d... robotToTags) {
		return score(prepareToScore(tagID, level, pickup, robotToTags), level, retreatDistance);
	}

	/**
	 * Returns a {@code Command} to score at the closest {@code AprilTag} and the
	 * specified level.
	 * 
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param retreatDistance the retreat distance at the end of scoring
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to score at the closest {@code AprilTag} and the
	 *         specified level
	 */
	public static Command scoreClosest(int level, boolean pickup, double retreatDistance, Transform2d... robotToTags) {
		return score(prepareToScoreClosest(level, pickup, robotToTags), level, retreatDistance);
	}

	/**
	 * Returns a {@code Command} to score at the specified level.
	 * 
	 * @param prepare a {@code Command} that prepares the robot to score
	 * @param level the scoring level
	 * @param retreatDistance the retreat distance at the end of scoring
	 * @return a {@code Command} to score at the specified level
	 */
	private static Command score(Command prepare, int level, double retreatDistance) {
		var p = new ParallelCommandGroup();
		if (level == 4)
			p.addCommands(m_wristSubsystem.goToAngle(kGrabberAngleLevelFour - 10));
		if (retreatDistance > 0)
			p.addCommands(moveStraight(-retreatDistance, 0.16, 16)); // TODO: Optimize
		return sequence(prepare, m_cheeseStickSubsystem.release(0.7), p); // TODO: Check
	}

	/**
	 * Returns a {@code Command} to obtain a coral at the specified station, align
	 * to the specified {@code AprilTag}, and then score at the specified level.
	 * 
	 * @param tagIDStation the ID of the {@code AprilTag} at the coral station
	 * @param tagID the ID of the {@code AprilTag} to align to in order to score
	 * @param level the target scoring level
	 * @param pickup a {@code boolean} value indicating whether or not to pick up
	 *        the coral in the pocket
	 * @param retreatDistance the retreat distance at the end of scoring
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        target {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to obtain a coral at the specified station, align
	 *         to the specified {@code AprilTag}, and then score at the specified
	 *         level
	 */
	private static Command score(int tagIDStation, int tagID, int level, boolean pickup, double retreatDistance,
			Transform2d... robotToTags) {
		return sequence(
				m_cheeseStickSubsystem.grab(),
				toStation(tagIDStation),
				waitSeconds(2),
				pickupAtCoralStation(),
				score(tagID, level, pickup, retreatDistance, robotToTags));
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

	private static Command getOneScoreAndAlgae(int tagID) {
		return getOneScoreAndAlgae(() -> tagID);
	}

	private static Command getOneScoreAndAlgae(Supplier<Integer> tagID) {
		return sequence(
				score(tagID, 4, false, 0, kRobotToTagsLeft),
				removeAlgaeLevelTwo(tagID),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.16, 16)));
	}

	static Command getMiddleScoreAndAlgaeBlue() {
		return getOneScoreAndAlgae(21)
				.withName("Middle Score and Algae Blue");
	}

	static Command getMiddleScoreAndAlgaeRed() {
		return getOneScoreAndAlgae(10)
				.withName("Middle Score and Algae Red");
	}

	static Command getMiddleScoreAndAlgaePracticeField() {
		return getOneScoreAndAlgae(() -> m_poseEstimationSubsystem.closestTagID())
				.withName("Middle Score and Algae Practice Field");
	}

	static Command getLeftScoreAndAlgaeBlue() {
		return getOneScoreAndAlgae(20)
				.withName("Left Score and Algae Blue");
	}

	static Command getRightScoreAndAlgaeBlue() {
		return getOneScoreAndAlgae(22)
				.withName("Right Score and Algae Blue");
	}

	static Command getLeftScoreAndAlgaeRed() {
		return getOneScoreAndAlgae(11)
				.withName("Left Score and Algae Red");
	}

	static Command getRightScoreAndAlgaeRed() {
		return getOneScoreAndAlgae(9)
				.withName("Right Score and Algae Red");
	}

	public static Command getTwoScore(int tagID1, int tagIDStation, int tagID2) {
		return sequence(
				score(tagID1, 4, false, 0.5, kRobotToTagsRight),
				score(tagIDStation, tagID2, 4, false, 0.5, kRobotToTagsLeft),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getLeftTwoScoreBlue() {
		return getTwoScore(20, 13, 19)
				.withName("Blue-Left | Two Score ");
	}

	public static Command getRightTwoScoreBlue() {
		return getTwoScore(22, 12, 17)
				.withName("Blue-Right | Two Score ");
	}

	public static Command getLeftTwoScoreRed() {
		return getTwoScore(11, 1, 6)
				.withName("Red-Left | Two Score ");
	}

	public static Command getRightTwoScoreRed() {
		return getTwoScore(9, 2, 8)
				.withName("Red-Right | Two Score ");
	}

	public static Command getTwoScoreAndAlgae(int tagID1, int tagIDStation, int tagID2) {
		return sequence(
				score(tagID1, 4, false, 0.5, kRobotToTagsRight),
				score(tagIDStation, tagID2, 4, false, 0.5, kRobotToTagsLeft),
				removeAlgaeLevelTwo(tagID2),
				parallel(
						m_wristSubsystem.goToAngle(268),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getLeftTwoScoreAndAlgaeBlue() {
		return getTwoScoreAndAlgae(20, 13, 19)
				.withName("Blue-Left | Two Score and Algae ");
	}

	public static Command getRightTwoScoreAndAlgaeBlue() {
		return getTwoScoreAndAlgae(22, 12, 17)
				.withName("Blue-Right | Two Score and Algae ");
	}

	public static Command getLeftTwoScoreAndAlgaeRed() {
		return getTwoScoreAndAlgae(11, 1, 6)
				.withName("Red-Left | Two Score and Algae ");
	}

	public static Command getRightTwoScoreAndAlgaeRed() {
		return getTwoScoreAndAlgae(9, 2, 8)
				.withName("Red-Right | Two Score and Algae ");
	}

	public static Command getThreeScore(int tagID1, int tagIDStation, int tagID2, int tagID3) {
		return sequence(
				score(tagID1, 4, false, 0.5, kRobotToTagsRight),
				score(tagIDStation, tagID2, 4, false, 0.5, kRobotToTagsLeft),
				score(tagIDStation, tagID3, 4, false, 0.5, kRobotToTagsLeft),
				parallel(
						m_wristSubsystem.goToAngle(270),
						moveStraight(-0.5, 0.01, 1)));
	}

	public static Command getLeftThreeScoreBlue() {
		return getThreeScore(20, 13, 19, 18)
				.withName("Blue-Left | Three Score ");
	}

	public static Command getRightThreeScoreBlue() {
		return getThreeScore(22, 12, 17, 18)
				.withName("Blue-Right | Three Score ");
	}

	public static Command getLeftThreeScoreRed() {
		return getThreeScore(11, 1, 6, 7)
				.withName("Red-Left | Three Score ");
	}

	public static Command getRightThreeScoreRed() {
		return getThreeScore(9, 2, 8, 7)
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
	 * @param distance the distance in meters
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * 
	 * @return a {@code Command} for moving forward and then backward.
	 */
	public static Command moveForwardBackward(double distance, double distanceTolerance,
			double angleTolerance) {
		return sequence(
				moveStraight(distance, distanceTolerance, angleTolerance),
				waitSeconds(2),
				moveStraight(-distance, distanceTolerance, angleTolerance));
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
		Supplier<Pose2d> s = () -> m_driveSubsystem.getPose()
				.plus(transform(sideLength, 0, 90));
		return new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, distanceTolerance,
				angleTolerance, List.of(s, s, s, s));
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
		return toClosestTag(0, robotToTags);
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag}.
	 *
	 * @param forwardAdjustment the additional distance to move forward/backward
	 *        (positive: closer to the tag)
	 * @param robotToTags the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @return a {@code Command} to automatically align the robot to the closest
	 *         {@code AprilTag}
	 */
	public static Command toClosestTag(double forwardAdjustment, Transform2d... robotToTags) {
		return toTag(() -> m_poseEstimationSubsystem.closestTagID(), forwardAdjustment, robotToTags);
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
		return toTag(tagID, 0, robotToTags);
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
	public static Command toTag(int tagID, double forwardAdjustment, Transform2d... robotToTags) {
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
					forwardAdjustment, 0.0, r);
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
}