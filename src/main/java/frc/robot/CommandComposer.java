package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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

	private static Command scoreLevel(Supplier<Command> levelCommand) {
		return sequence(
				levelCommand.get(),
				m_wristSubsystem.goToAngle(35),
				m_elevatorSubsystem.lowerToScore(),
				m_cheeseStickSubsystem.goLeft(), // TODO: Left is release?
				m_cheeseStickSubsystem.goRight(),
				levelCommand.get(),
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

	public static Command prepareForCoralPickup() {
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(-90));
	}

	public static Command pickupAtCoralStation() {
		return sequence(
				m_cheeseStickSubsystem.goLeft(),
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_cheeseStickSubsystem.goRight());
	}

}