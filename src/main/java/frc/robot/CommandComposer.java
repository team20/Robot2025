package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CheeseStickSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static AlgaeGrabberSubsystem m_algaeGrabberSubsystem;
	private static CheeseStickSubsystem m_cheeseStickSubsystem;
	private static ClimberSubsystem m_climberSubsystem;
	private static ElevatorSubsystem m_elevatorSubsystem;
	private static WristSubsystem m_wristSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			AlgaeGrabberSubsystem algaeGrabberSubsystem,
			CheeseStickSubsystem cheeseStickSubsystem,
			ClimberSubsystem climberSubsystem,
			ElevatorSubsystem elevatorSubsystem,
			WristSubsystem wristSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_algaeGrabberSubsystem = algaeGrabberSubsystem;
		m_cheeseStickSubsystem = cheeseStickSubsystem;
		m_climberSubsystem = climberSubsystem;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;
	}

	private static Command scoreLevelInTeleop(double level, Supplier<Command> levelCommand,
			double wristAngle) {
		return sequence(
				m_elevatorSubsystem.goToClearanceHeight(level),
				m_wristSubsystem.goToAngle(wristAngle),
				levelCommand.get());
	}

	public static Command scoreLevelOneInTeleop() {
		return scoreLevelInTeleop(kLevelOneHeight, m_elevatorSubsystem::goToLevelOneHeight, kGrabberAngleOthers);
	}

	public static Command scoreLevelTwoInTeleop() {
		return scoreLevelInTeleop(kLevelTwoHeight, m_elevatorSubsystem::goToLevelOneHeight, kGrabberAngleOthers);
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
		return sequence(
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
				m_driveSubsystem.driveCommand(z, z, z, () -> 1, f).withTimeout(duration), // 90 degrees
				m_driveSubsystem.driveCommand(z, z, () -> -1, z, f).withTimeout(duration), // 180 degrees
				m_driveSubsystem.driveCommand(z, z, z, () -> -1, f).withTimeout(duration), // 270 degrees
				m_driveSubsystem.driveCommand(z, z, () -> 1, () -> 1, f).withTimeout(duration), // 45 degrees
				m_driveSubsystem.driveCommand(z, z, () -> 1, z, f).withTimeout(duration)) // 0 degrees
		;
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
}