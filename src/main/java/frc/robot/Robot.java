// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.PathDriveCommand;
import frc.robot.simulation.VisionSimulator;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CheeseStickSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final SendableChooser<Command> m_testingChooser = new SendableChooser<>();
	private final Mechanism2d m_mechanism = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(100));
	private final AlgaeGrabberSubsystem m_algaeGrabberSubsystem = new AlgaeGrabberSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(
			m_mechanism.getRoot("anchor", Units.inchesToMeters(23), 0));
	private final WristSubsystem m_wristSubsystem = new WristSubsystem(m_elevatorSubsystem.getWristMount());
	private final CheeseStickSubsystem m_cheeseStickSubsystem = new CheeseStickSubsystem(
			m_wristSubsystem.getCheeseStickMount());
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(kOperatorControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	// private final VisionSimulator m_visionSimulator = new
	// VisionSimulator(m_driveSubsystem,
	// pose(kFieldLayout.getFieldLength() / 2 - 1.5, 1.91 + .3, 180), 0.01);
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem,
			pose(kFieldLayout.getFieldLength() / 2 + 1.5, 1.91 + .3, 0), 0.01);
	SimCameraProperties cameraProp = new SimCameraProperties() {
		{
			setCalibration(640, 480, Rotation2d.fromDegrees(100));
			// Approximate detection noise with average and standard deviation error in
			// pixels.
			setCalibError(0.1, 0.05);
			// Set the camera image capture framerate (Note: this is limited by robot loop
			// rate).
			setFPS(20);
			// The average and standard deviation in milliseconds of image data latency.
			setAvgLatencyMs(35);
			setLatencyStdDevMs(5);

		}
	};
	private final PhotonCamera m_camera1 = RobotBase.isSimulation()
			? cameraSim("Camera1", kRobotToCamera1, m_visionSimulator, cameraProp)
			: new PhotonCamera("Cool camera");
	private final PhotonCamera m_camera2 = RobotBase.isSimulation()
			? cameraSim("Camera2", kRobotToCamera2, m_visionSimulator, cameraProp)
			: new PhotonCamera("Cool camera2");
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_driveSubsystem)
			.addCamera(m_camera1, kRobotToCamera1)
			.addCamera(m_camera2, kRobotToCamera2);

	public Robot() {
		CommandComposer.setSubsystems(
				m_driveSubsystem, m_algaeGrabberSubsystem, m_cheeseStickSubsystem, m_climberSubsystem,
				m_elevatorSubsystem, m_wristSubsystem, m_poseEstimationSubsystem);
		// var dropChute = new MechanismLigament2d("bottom", Units.inchesToMeters(5), 0,
		// 5, new Color8Bit(Color.kBeige));
		// dropChute.append(new MechanismLigament2d("side", Units.inchesToMeters(12),
		// 90, 5, new Color8Bit(Color.kWhite)));
		// m_mechanism.getRoot("dropChute", Units.inchesToMeters(28),
		// Units.inchesToMeters(9)).append(dropChute);
		SmartDashboard.putData("Superstructure", m_mechanism);
		SmartDashboard.putData(m_pdh);
		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(
				Map.of(
						11, "FR Turn", 21, "BR Turn", 31, "BL Turn", 41, "FL Turn", kElevatorMotorPort, "Elevator",
						kClimberMotorPort, "Climber Motor", kWristMotorPort, "Wrist Motor", kFlywheelMotorPort,
						"Algae Flywheel Motor", kGrabberAnglePort, "Algae Pivot Motor"));
		DriverStation.startDataLog(DataLogManager.getLog());
		addAutoCommands();
		addTestingCommands();
		addProgrammingCommands();
		bindClimberControls();
		bindDriveControls();
		bindElevatorControls();
		bindWristControls();
		bindAlgaeControls();
		bindCheeseStickControls();
		SmartDashboard.putData("Auto Selector", m_autoSelector);
		SmartDashboard.putData("Testing Chooser", m_testingChooser);
		m_driverController.options().and(m_driverController.create()).and(() -> !DriverStation.isFMSAttached())
				.onTrue(Commands.deferredProxy(m_testingChooser::getSelected));
	}

	public void addAutoCommands() {
		m_autoSelector
				.addOption(
						"3 Score South", CommandComposer.get3ScoreSouth());
	}

	public void addTestingCommands() {
		m_testingChooser
				.addOption(
						"Check All Subsystems",
						parallel(
								m_cheeseStickSubsystem.testCommand(2.0), m_wristSubsystem.testCommand(2.0),
								m_elevatorSubsystem.testCommand(3.0),
								m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0)));
		m_testingChooser
				.addOption(
						"Check CheeseStickSubsystem",
						m_cheeseStickSubsystem.testCommand(2.0));
		m_testingChooser
				.addOption(
						"Check WristSubsystem",
						m_wristSubsystem.testCommand(2.0));
		m_testingChooser
				.addOption(
						"Check ElevatorSubsystem (Levels 0, 1, 0, 3, 2, 4, and 0)",
						m_elevatorSubsystem.testCommand(2.0));
		m_testingChooser
				.addOption(
						"Check DriveSubsystem (F/B/L/R/LR/RR and F/B while rotating)",
						m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0));
		double distanceTolerance = 0.01;
		double angleToleranceInDegrees = 1;
		double intermediateDistanceTolerance = 0.08;
		double intermediateAngleToleranceInDegrees = 8.0;
		m_testingChooser
				.addOption(
						"Check PID Constants for Driving (5'x5' Square)",
						CommandComposer
								.moveOnSquare(Units.feetToMeters(5), distanceTolerance, angleToleranceInDegrees, 16));
		m_testingChooser
				.addOption(
						"Quickly Align to AprilTags 12, 13, 17, 18, and 19",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 18,
								17, 12, 17, 18, 19, 13, 19, 18));
		m_testingChooser
				.addOption(
						"Quickly Align AprilTags 17, 18, 19, 20, 21, and 22",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTagsLeft),
								kRobotToTagsLeft[0], 17, 18, 19, 20, 21, 22, 17));
		m_testingChooser
				.addOption(
						"Quickly Align to AprilTags 1, 2, 6, 7, and 8",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees, Arrays.asList(kRobotToTags), kRobotToTags[0], 7, 6,
								1, 6, 7, 8, 2, 8, 7));
		m_testingChooser
				.addOption(
						"Check kDriveGearRatio and kWheelDiameter (F/B 6 feet)",
						CommandComposer.moveForwardBackward(6, distanceTolerance, angleToleranceInDegrees));
		m_testingChooser
				.addOption(
						"Slowest Movement Test (F/B/L/R/LR/RR and F/B while rotating)",
						m_driveSubsystem.testCommand(kDriveMinSpeed, kTurnMinAngularSpeed, 1.0));
		m_testingChooser
				.addOption(
						"Fastest Forward/Backward Movement Test (5m)",
						sequence(
								CommandComposer.moveStraight(5, 0.1, 10),
								CommandComposer.moveStraight(-5, 0.1, 10)));
		m_testingChooser
				.addOption(
						"Fastest Rotation Test (5 rotations)",
						new PathDriveCommand(m_driveSubsystem, 1, 10,
								1, 100,
								IntStream.range(1, 1 + 3 * 5)
										.mapToObj(
												i -> (Supplier<Pose2d>) (() -> {
													var pose = m_driveSubsystem.getPose();
													return pose(pose.getX(), pose.getY(), 120 * i);
												}))
										.toList()));
	}

	public void addProgrammingCommands() {
		m_testingChooser
				.addOption("SysId Drive Quasistatic Forward", m_driveSubsystem.sysidQuasistatic(Direction.kForward));
		m_testingChooser
				.addOption("SysId Drive Quasistatic Reverse", m_driveSubsystem.sysidQuasistatic(Direction.kReverse));
		m_testingChooser.addOption("SysId Drive Dynamic Forward", m_driveSubsystem.sysidDynamic(Direction.kForward));
		m_testingChooser.addOption("SysId Drive Dynamic Reverse", m_driveSubsystem.sysidDynamic(Direction.kReverse));
		m_testingChooser
				.addOption("SysId Wrist Quasistatic Forward", m_wristSubsystem.sysidQuasistatic(Direction.kForward));
		m_testingChooser
				.addOption("SysId Wrist Quasistatic Reverse", m_wristSubsystem.sysidQuasistatic(Direction.kReverse));
		m_testingChooser.addOption("SysId Wrist Dynamic Forward", m_wristSubsystem.sysidDynamic(Direction.kForward));
		m_testingChooser.addOption("SysId Wrist Dynamic Reverse", m_wristSubsystem.sysidDynamic(Direction.kReverse));
		m_testingChooser.addOption(
				"SysId Elevator Quasistatic Forward", m_elevatorSubsystem.sysidQuasistatic(Direction.kForward));
		m_testingChooser.addOption(
				"SysId Elevator Quasistatic Reverse", m_elevatorSubsystem.sysidQuasistatic(Direction.kReverse));
		m_testingChooser
				.addOption("SysId Elevator Dynamic Forward", m_elevatorSubsystem.sysidDynamic(Direction.kForward));
		m_testingChooser
				.addOption("SysId Elevator Dynamic Reverse", m_elevatorSubsystem.sysidDynamic(Direction.kReverse));
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						// (m_driverController.axisMagnitudeGreaterThan(1, 5) == 0) ?
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getSquareButton)); // makes the robot robot-oriented
		// m_driveSubsystem.setDefaultCommand(
		// m_driveSubsystem.driveCommand(
		// () -> -m_driverController.getLeftY(),
		// () -> -m_driverController.getLeftX(),
		// () -> -m_driverController.getRightY(),
		// () -> -m_driverController.getRightX(),
		// m_driverController.getHID()::getSquareButton)); // makes the robot
		// robot-oriented

		m_driverController.options().onTrue(m_driveSubsystem.resetHeading());

		m_driverController.square()
				.whileTrue(
						CommandComposer.scoreLevelTwo(CommandComposer.toClosestTag(kRobotToTagsLeft)));
		m_driverController.cross()
				.whileTrue(CommandComposer.scoreLevelTwo(CommandComposer.toClosestTag(kRobotToTagsRight)));

		// m_operatorController.povLeft().whileTrue(
		// CommandComposer.driveWithLeftAlignment(
		// () -> -m_driverController.getLeftY(),
		// () -> -m_driverController.getLeftX(),
		// () -> m_driverController.getL2Axis() - m_driverController.getR2Axis()));

		// m_operatorController.povRight().whileTrue(
		// CommandComposer.driveWithRightAlignment(
		// () -> -m_driverController.getLeftY(),
		// () -> -m_driverController.getLeftX(),
		// () -> m_driverController.getL2Axis() - m_driverController.getR2Axis()));

	}

	public void bindElevatorControls() {
		RobotModeTriggers.disabled().onTrue(m_elevatorSubsystem.stopMotor());
		m_operatorController.axisMagnitudeGreaterThan(PS5Controller.Axis.kLeftY.value, ControllerConstants.kDeadzone)
				.whileTrue(m_elevatorSubsystem.manualMove(() -> -m_operatorController.getLeftY()));
		m_operatorController.triangle().onTrue(
				m_elevatorSubsystem.goToLevelFourHeight().andThen(m_wristSubsystem.goToAngle(kGrabberAngleLevelFour)));
		m_operatorController.square().onTrue(
				m_elevatorSubsystem.goToLevelThreeHeight().andThen(m_wristSubsystem.goToAngle(kGrabberAngleOthers)));
		m_operatorController.cross().onTrue(
				m_elevatorSubsystem.goToLevelTwoHeight().andThen(m_wristSubsystem.goToAngle(kGrabberAngleOthers)));
		m_operatorController.circle().onTrue(
				m_elevatorSubsystem.goToLevelOneHeight().andThen(m_wristSubsystem.goToAngle(kGrabberAngleOthers)));
		// m_operatorController.povLeft().onTrue(m_elevatorSubsystem.goToCoralStationHeight());
		m_operatorController.L1().and(m_operatorController.circle()).onTrue(m_elevatorSubsystem.goToBaseHeight());
		m_operatorController.create().onTrue(m_elevatorSubsystem.resetTheEncoder());
	}

	public void bindAlgaeControls() {
		// m_algaeGrabberSubsystem
		// .setDefaultCommand(m_algaeGrabberSubsystem.manualMove(() ->
		// m_operatorController.getRightY()));
		m_operatorController.L2().onTrue(m_algaeGrabberSubsystem.grabAlgaeAndHold());
		m_operatorController.R2().onTrue(m_algaeGrabberSubsystem.releaseAlgae());

		m_operatorController.options().onTrue(m_algaeGrabberSubsystem.reverseFlywheelAndStop());
	}

	public void bindWristControls() {
		m_wristSubsystem.setDefaultCommand(m_wristSubsystem.manualMove(() -> m_operatorController.getRightY()));
		// m_driverController.circle().onTrue(m_wristSubsystem.reverseMotor());
		// m_driverController.square().onTrue(m_wristSubsystem.forwardMotor());
	}

	public void bindCheeseStickControls() {
		m_operatorController.R1().whileFalse(m_cheeseStickSubsystem.grab());
		m_operatorController.R1().whileTrue(m_cheeseStickSubsystem.release());
	}

	public void bindClimberControls() {
		// m_climberSubsystem.setDefaultCommand(m_climberSubsystem.manualMove(() ->
		// m_operatorController.getRightY()));

		// m_operatorController.povDown().whileTrue(m_climberSubsystem.moveForward())
		// .onFalse(m_climberSubsystem.moveBackward());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
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
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
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
		var testCommand = m_testingChooser.getSelected();
		if (testCommand != null)
			testCommand.schedule();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	/**
	 * Constructs a {@code PhotonCamera} that provides simulation.
	 * 
	 * @param cameraName the name of the {@code PhotonCamera}
	 * @param robotToCamera the {@code Pose2d} of the {@code PhotonCamera} relative
	 *        to the center of the robot
	 * @param m_visionSimulator the {@code VisionSimulator} to use
	 * @param cameraProp the {@code SimCameraProperties} to use
	 * @return the constructed {@code PhotonCamera}
	 */
	PhotonCamera cameraSim(String cameraName, Transform3d robotToCamera, VisionSimulator m_visionSimulator,
			SimCameraProperties cameraProp) {
		PhotonCamera camera = new PhotonCamera(cameraName);
		PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
		cameraSim.enableProcessedStream(true);
		cameraSim.enableDrawWireframe(true);
		m_visionSimulator.addCamera(cameraSim, robotToCamera);
		return camera;
	}

}
