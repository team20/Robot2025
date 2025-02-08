// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.AlgaeGrabberSubsystem.GrabberState;
import frc.robot.subsystems.CheeseStickSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonCameraSimulator;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSimulator;
import frc.robot.subsystems.WristSubsystem;

public class Robot extends TimedRobot {
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final SendableChooser<Command> m_testSelector = new SendableChooser<Command>();

	private Command m_autonomousCommand;
	private final Mechanism2d m_mechanism = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(100));
	private final AlgaeGrabberSubsystem m_algaeGrabberSubsystem = new AlgaeGrabberSubsystem();
	private final CheeseStickSubsystem m_cheeseStickSubsystem = new CheeseStickSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private Command m_testCommand;
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(
			m_mechanism.getRoot("anchor", Units.inchesToMeters(23), 0));
	private final WristSubsystem m_wristSubsystem = new WristSubsystem(m_elevatorSubsystem.getWristMount());
	// Changed to PS5
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(kOperatorControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem,
			pose(kFieldLayout.getFieldLength() / 2, 1.91, 0), 0.01);
	private final PhotonCamera m_camera1 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera1", kRobotToCamera1, m_visionSimulator, 3, 0.1)
			: new PhotonCamera("Cool camera");
	private final PhotonCamera m_camera2 = RobotBase.isSimulation()
			? new PhotonCameraSimulator("Camera2", kRobotToCamera2, m_visionSimulator, 3, 0.1)
			: new PhotonCamera("Cool camera2");
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_driveSubsystem)
			.addCamera(m_camera1, kRobotToCamera1)
			.addCamera(m_camera2, kRobotToCamera2);

	public Robot() {
		var dropChute = new MechanismLigament2d("bottom", Units.inchesToMeters(5), 0, 5, new Color8Bit(Color.kBeige));
		dropChute.append(new MechanismLigament2d("side", Units.inchesToMeters(12), 90, 5, new Color8Bit(Color.kWhite)));
		m_mechanism.getRoot("dropChute", Units.inchesToMeters(28), Units.inchesToMeters(9)).append(dropChute);
		SmartDashboard.putData("Superstructure", m_mechanism);
		CommandComposer.setSubsystems(m_driveSubsystem, m_poseEstimationSubsystem);

		m_autoSelector.addOption("Test DriveSubsystem", m_driveSubsystem.testCommand());

		m_testSelector.addOption("Test DriveSubsystem", m_driveSubsystem.testCommand());
		m_testSelector.addOption("Test Rotation", CommandComposer.testRotation());
		m_testSelector.addOption("Turn toward Tag 1", CommandComposer.turnTowardTag(1));

		SmartDashboard.putData("Auto Selector", m_autoSelector);
		SmartDashboard.putData("Test Selector", m_testSelector);

		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		var m = new LinkedHashMap<Integer, String>(Map.of(
				10, "FR Drive", 11, "FR Turn", 20, "BR Drive", 21, "BR Turn", 30, "BL Drive", 31, "BL Turn",
				40, "FL Drive", 41, "FL Turn"));
		m.putAll(
				Map.of(
						kElevatorMotorPort, "Elevator",
						kClimberMotorPort, "Climber Motor", kWristMotorPort, "Wrist Motor", kFlywheelMotorPort,
						"Algae Motor"));
		URCL.start(m);
		DriverStation.startDataLog(DataLogManager.getLog());
		bindDriveControls();
		bindElevatorControls();
		bindWristControls();
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getSquareButton));
		// TODO: Add in joystick rotation
		m_driverController.button(Button.kSquare)
				.whileTrue(
						driveWithAlignmentCommand(
								() -> -m_driverController.getLeftY(),
								() -> -m_driverController.getLeftX(),
								() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
								new Transform2d(0.5, 0, Rotation2d.fromDegrees(180)), 2));
	}

	public void bindElevatorControls() {
		m_operatorController.circle().onTrue(m_elevatorSubsystem.goToLevelFourCommand());
		m_operatorController.triangle().onTrue(m_elevatorSubsystem.goToLevelThreeCommand());
		m_operatorController.square().onTrue(m_elevatorSubsystem.goToLevelTwoCommand());
		m_operatorController.cross().onTrue(m_elevatorSubsystem.goToLevelOneCommand());
		m_operatorController.povLeft().onTrue(m_elevatorSubsystem.goToCoralStationCommand());
		// TODO: Add manual movement
	}

	public void bindAlgaeControls() {
		m_operatorController.R1().onTrue(
				m_algaeGrabberSubsystem.deployGrabberCommand(GrabberState.DOWN)
						.andThen(m_algaeGrabberSubsystem.runFlywheelCommand())); // TODO: Come up after?
		m_operatorController.L1().onTrue(
				m_algaeGrabberSubsystem.runFlywheelReverseCommand()
						.until(m_algaeGrabberSubsystem::checkCurrentOnFlywheel)
						.andThen(m_algaeGrabberSubsystem.stopFlywheelCommand()));
	}

	public void bindWristControls() {
		m_driverController.circle().onTrue(m_wristSubsystem.reverseMotor());
		m_driverController.square().onTrue(m_wristSubsystem.forwardMotor());
	}

	public void bindCheeseStickControls() {
		// Need to figure out what left and right actually mean
		m_operatorController.R2().whileFalse(m_cheeseStickSubsystem.goLeft());
		m_operatorController.R2().whileTrue(m_cheeseStickSubsystem.goRight());
	}

	public void bindClimberControls() {
		m_operatorController.L2().whileTrue(m_climberSubsystem.moveForward())
				.onFalse(m_climberSubsystem.moveBackward());
	}

	/**
	 * Creates a {@code Command} to automatically align the robot to the closest
	 * {@code AprilTag} while driving the robot with joystick input.
	 *
	 * @param forwardSpeed forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation rotation speed supplier. Positive values make the
	 *        robot rotate CCW.
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        closest {@code AprilTag} relative to the robot when the robot is
	 *        aligned
	 * @param isFieldRelative {@code Supplier} for determining whether or not
	 *        driving should be field relative.
	 * @return a {@code Command} to automatically align the robot to the closest tag
	 *         while driving the robot with joystick input
	 */
	Command driveWithAlignmentCommand(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation, Transform2d robotToTag, double distanceThresholdInMeters) {

		return run(() -> {
			ChassisSpeeds speeds = DriveSubsystem.chassisSpeeds(forwardSpeed, strafeSpeed, rotation);
			speeds = speeds.plus(
					m_poseEstimationSubsystem
							.chassisSpeedsTowardClosestTag(robotToTag, distanceThresholdInMeters));
			m_driveSubsystem.drive(speeds, true);
		});
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_autoSelector.getSelected();
		if (m_autonomousCommand != null)
			m_autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null)
			m_autonomousCommand.cancel();
		if (m_testCommand != null)
			m_testCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		m_testCommand = m_testSelector.getSelected();
		if (m_testCommand != null)
			m_testCommand.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}