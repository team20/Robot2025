package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
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

}