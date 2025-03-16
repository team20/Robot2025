package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
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
	private static ArduinoSubsystem m_arduinoSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			AlgaeGrabberSubsystem algaeGrabberSubsystem,
			CheeseStickSubsystem cheeseStickSubsystem,
			ClimberSubsystem climberSubsystem,
			ElevatorSubsystem elevatorSubsystem,
			WristSubsystem wristSubsystem, ArduinoSubsystem arduinoSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_algaeGrabberSubsystem = algaeGrabberSubsystem;
		m_cheeseStickSubsystem = cheeseStickSubsystem;
		m_climberSubsystem = climberSubsystem;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_arduinoSubsystem = arduinoSubsystem;
	}

	/**
	 * Prepares for scoring in teleop | no release & back to base
	 * 
	 * @param level the level you want to go to (for the clerance calculation)
	 * @param clearanceHeight the number that is needed to be added onto the level
	 *        to clear the reef / intake
	 * @param levelCommand the actual level to go to
	 * @param wristAngle the angle the wrist needs to go to
	 * @return the command in sequence | clearance, wrist angle, level height
	 */
	private static Command scoreLevelInTeleop(double level, double clearanceHeight, Supplier<Command> levelCommand,
			double wristAngle) {
		return sequence(
				m_elevatorSubsystem.goToClearanceHeight(level, Units.inchesToMeters(clearanceHeight)),
				m_wristSubsystem.goToAngle(wristAngle),
				levelCommand.get());
	}

	/**
	 * TELEOP | Level 1
	 * 
	 * @return uses {@link #scoreLevelInTeleop} to score in level 1
	 */
	public static Command scoreLevelOneInTeleop() {
		return scoreLevelInTeleop(
				kLevelOneHeight, kLevelOneClearanceHeight, m_elevatorSubsystem::goToLevelOneHeight,
				kGrabberAngleLevelOneAndTwo)
						.withName("Score Level One in Teleop");
	}

	/**
	 * TELEOP | Level 2
	 * 
	 * @return uses {@link #scoreLevelInTeleop} to score in level 2
	 */
	public static Command scoreLevelTwoInTeleop() {
		return scoreLevelInTeleop(
				kLevelTwoHeight, kLevelTwoClearanceHeight, m_elevatorSubsystem::goToLevelTwoHeight,
				kGrabberAngleLevelOneAndTwo)
						.withName("Score Level Two in Teleop");
	}

	/**
	 * TELEOP | Level 3 -> NO CLEARANCE
	 * 
	 * @return uses {@link #scoreLevelInTeleop} to score in level 3
	 */
	public static Command scoreLevelThreeInTeleop() {
		return sequence(
				// m_elevatorSubsystem.goToLevelThreeHeight(),
				// m_wristSubsystem.goToAngle(kGrabberAngleLevelThree))
				scoreLevelInTeleop(
						kGrabberAngleLevelThree, 0, m_elevatorSubsystem::goToLevelThreeHeight, kGrabberAngleLevelThree))
								.withName("Elevator to Level Three and Wrist to Angle");
	}

	/**
	 * TELEOP | Level 4 -> NO CLEARANCE
	 * 
	 * @return uses {@link #scoreLevelInTeleop} to score in level 4
	 */
	public static Command scoreLevelFourInTelop() {
		return sequence(
				// m_elevatorSubsystem.goToLevelFourHeight(),
				// m_wristSubsystem.goToAngle(kGrabberAngleLevelFour))
				scoreLevelInTeleop(
						kGrabberAngleLevelFour, 0, m_elevatorSubsystem::goToLevelFourHeight, kGrabberAngleLevelFour))
								.withName("Elevator to Level Four and Wrist to Angle");
	}

	/**
	 * Remove High Algae | Level Three
	 * 
	 * @return sequence | level two height -> wrist -> algae three height
	 */
	public static Command removeAlgaeLevelThree() {
		return sequence(
				m_elevatorSubsystem.goToLevelTwoHeight(),
				m_wristSubsystem.goToAngle(kAlgaeAngle),
				m_elevatorSubsystem.goToUpperAlgaeHeight()).withName("Remove Algae Level Three");
	}

	/**
	 * Remove Lower Algae | Level Two
	 * 
	 * @return sequence | coral station height -> wrist -> algae level two height
	 */
	public static Command removeAlgaeLevelTwo() {
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(kAlgaeAngle),
				m_elevatorSubsystem.goToLowerAlgaeHeight()).withName("Remove Algae Level Two");
	}

	/**
	 * Deploys the algae grabber and spins the flywheels, once current is tripped
	 * changes LEDs to INTAKED_ALGAE
	 * 
	 * @return the sequence
	 */
	public static Command grabAlgaeAndLEDs() {
		return sequence(
				m_algaeGrabberSubsystem.grabAlgaeAndHold(),
				m_arduinoSubsystem.ledPattern(StatusCode.INTAKED_ALGAE))
						.withName("Grab Algae and Hold");
	}

	/**
	 * Drives incredibly slow forward (elevator has to be forward)
	 * 
	 * @return {@link frc.robot.subsystems.DriveSubsystem#driveCommand()} at 0.25
	 *         speed
	 */
	public static Command leave() {
		return m_driveSubsystem.driveCommand(() -> 0.25, () -> 0, () -> 0, () -> true).withTimeout(10)
				.withName("Leave Auto");
	}

	/**
	 * Brings the elevator up to coral acquisition height and the wrist to the
	 * plunge angle
	 * 
	 * @return the sequence
	 */
	public static Command prepareForCoralPickup() { // TODO: Make parallel
		return sequence(
				m_elevatorSubsystem.goToCoralStationHeight(),
				m_wristSubsystem.goToAngle(kBaseAngle)).withName("Prepare For Coral Pickup");
	}

	/**
	 * Wrist to 270 and then the elevator to base position
	 * 
	 * @return the sequence
	 */
	public static Command goToBase() {
		return sequence(
				m_wristSubsystem.goToAngle(kBaseAngle),
				m_elevatorSubsystem.goToBaseHeight()).withName("Go To Base");
	}

	/**
	 * Retracts the climber and sets drive into coast so the climber can slide
	 * forward
	 * 
	 * @return in parallel
	 */
	public static Command retractClimber() {
		return parallel(m_climberSubsystem.retract(), m_driveSubsystem.setNeutralMode(NeutralModeValue.Coast).asProxy())
				.finallyDo(() -> m_driveSubsystem.setDriveMotorNeutralMode(NeutralModeValue.Brake))
				.withName("Retract Climber and Drive Coast");
	}
}