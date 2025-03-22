// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.CommandComposer.*;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.AutoAlignConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.simulation.VisionSimulator;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
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
	private final WristSubsystem m_wristSubsystem = new WristSubsystem(m_elevatorSubsystem);
	private final CheeseStickSubsystem m_cheeseStickSubsystem = new CheeseStickSubsystem(
			m_wristSubsystem.getCheeseStickMount());
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(kOperatorControllerPort);
	private final PowerDistribution m_pdh = new PowerDistribution();
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem,
			pose(kFieldLayout.getFieldLength() / 2 + 1.5, kFieldLayout.getFieldWidth() / 2, 0), 0.01);
	SimCameraProperties cameraProp = new SimCameraProperties() {
		{
			setCalibration(640, 480, Rotation2d.fromDegrees(110));
			// Approximate detection noise with average and standard deviation error in
			// pixels.
			setCalibError(0.2, 0.1);
			// Set the camera image capture framerate (Note: this is limited by robot loop
			// rate).
			setFPS(20);
			// The average and standard deviation in milliseconds of image data latency.
			setAvgLatencyMs(35);
			setLatencyStdDevMs(5);

		}
	};
	private final PhotonCamera m_camera1 = RobotBase.isSimulation()
			? cameraSim("FrontCamera", kRobotToFrontCamera, m_visionSimulator, cameraProp)
			: new PhotonCamera("FrontCamera");
	private final PhotonCamera m_camera2 = RobotBase.isSimulation()
			? cameraSim("BackCamera", kRobotToBackCamera, m_visionSimulator, cameraProp)
			: new PhotonCamera("BackCamera");
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_driveSubsystem)
			.addCamera(m_camera1, kRobotToFrontCamera)
			.addCamera(m_camera2, kRobotToBackCamera);

	public Robot() {
		SignalLogger.start();
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
		CommandComposer.setSubsystems(
				m_driveSubsystem, m_algaeGrabberSubsystem, m_cheeseStickSubsystem, m_climberSubsystem,
				m_elevatorSubsystem, m_wristSubsystem, m_poseEstimationSubsystem);
		var dropChute = new MechanismLigament2d("bottom", Units.inchesToMeters(5), 0, 5, new Color8Bit(Color.kBeige));
		dropChute.append(new MechanismLigament2d("side", Units.inchesToMeters(12), 90, 5, new Color8Bit(Color.kWhite)));
		m_mechanism.getRoot("dropChute", Units.inchesToMeters(28), Units.inchesToMeters(9)).append(dropChute);
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
		bindAlert(
				new Alert("Driver Joystick Disconnected!", AlertType.kError), () -> !m_driverController.isConnected());
		bindAlert(
				new Alert("Operator Joystick Disconnected!", AlertType.kError),
				() -> !m_operatorController.isConnected());
		DriverStation.silenceJoystickConnectionWarning(true);
		SmartDashboard.putData("Testing Chooser", m_testingChooser);
		SmartDashboard.putData("Auto Selector", m_autoSelector);
		m_driverController.options().and(m_driverController.create()).and(() -> !DriverStation.isFMSAttached())
				.onTrue(Commands.deferredProxy(m_testingChooser::getSelected));
	}

	public void addAutoCommands() {
		m_autoSelector.addOption("Leave", leave());

		// One score and algae, center starting position
		m_autoSelector.addOption("Middle and Algae Blue (Tag 21)", getMiddleScoreAndAlgaeBlue());
		m_autoSelector.addOption("Middle and Algae Red (Tag 10)", getMiddleScoreAndAlgaeRed());

		// One score and algae, left or right starting position
		m_autoSelector.addOption("Left and Algae Blue (Tag 20)", getLeftScoreAndAlgaeBlue());
		m_autoSelector.addOption("Right and Algae Blue (Tag 22)", getRightScoreAndAlgaeBlue());
		m_autoSelector.addOption("Left and Algae Red (Tag 11)", getLeftScoreAndAlgaeRed());
		m_autoSelector.addOption("Right and Algae Red (Tag 9)", getRightScoreAndAlgaeRed());

		// Two score, left or right starting position
		m_autoSelector.addOption("Left Two Score Blue (Tag 20)", getLeftTwoScoreBlue());
		m_autoSelector.addOption("Right Two Score Blue (Tag 22)", getRightTwoScoreBlue());
		m_autoSelector.addOption("Left Two Score Red (Tag 11)", getLeftTwoScoreRed());
		m_autoSelector.addOption("Right Two Score Red (Tag 9)", getRightTwoScoreRed());

		// Two score and algae, left or right starting position
		m_autoSelector.addOption("Left Two Score and Algae Blue (Tag 20)", getLeftTwoScoreAndAlgaeBlue());
		m_autoSelector.addOption("Right Two Score and Algae Blue (Tag 22)", getRightTwoScoreAndAlgaeBlue());
		m_autoSelector.addOption("Left Two Score and Algae Red (Tag 11)", getLeftTwoScoreAndAlgaeRed());
		m_autoSelector.addOption("Right Two Score and Algae Red (Tag 9)", getRightTwoScoreAndAlgaeRed());

		// Three score, left or right starting position
		m_autoSelector.addOption("Left Three Score Blue (Tag 20)", getLeftThreeScoreBlue());
		m_autoSelector.addOption("Right Three Score Blue (Tag 22)", getRightThreeScoreBlue());
		m_autoSelector.addOption("Left Three Score Red (Tag 11)", getLeftThreeScoreRed());
		m_autoSelector.addOption("Right Three Score Red (Tag 9)", getRightThreeScoreRed());
	}

	public void addTestingCommands() {
		m_testingChooser
				.addOption(
						"Remove Algae (Closest)",
						selectIfConfident(removeAlgaeLevelTwo(() -> m_poseEstimationSubsystem.closestTagID())));
		m_testingChooser
				.addOption(
						"L4 Score Left and Right (Closest)",
						selectIfConfident(
								sequence(
										goToBase(),
										scoreClosest(4, false, 1.5, kRobotToTagsLeft),
										waitSeconds(2),
										goToBase(),
										scoreClosest(4, false, 1.5, kRobotToTagsRight))));
		m_testingChooser
				.addOption(
						"L3 Score Left and Right (Closest)",
						selectIfConfident(
								sequence(
										goToBase(),
										scoreClosest(3, false, 1.5, kRobotToTagsLeft),
										waitSeconds(2),
										goToBase(),
										scoreClosest(3, false, 1.5, kRobotToTagsRight))));
		m_testingChooser
				.addOption(
						"Score from Station (Tag 1)",
						selectIfConfident(
								sequence(score(1, 4, 6))));
		m_testingChooser
				.addOption(
						"Score from Station (Tag 2)",
						selectIfConfident(
								sequence(score(2, 4, 8))));
		m_testingChooser
				.addOption(
						"Score from Station (Tag 12)",
						selectIfConfident(
								sequence(score(12, 4, 17))));
		m_testingChooser
				.addOption(
						"Score from Station (Tag 13)",
						selectIfConfident(
								sequence(score(13, 4, 19))));
		m_testingChooser
				.addOption(
						"Score Far from Station (Tag 1)",
						selectIfConfident(
								sequence(score(1, 4, 8, 9, 10))));
		m_testingChooser
				.addOption(
						"Score Far from Station (Tag 2)",
						selectIfConfident(
								sequence(score(2, 4, 10, 11, 6))));
		m_testingChooser
				.addOption(
						"Score Far from Station (Tag 12)",
						selectIfConfident(
								sequence(score(12, 4, 19, 20, 21))));
		m_testingChooser
				.addOption(
						"Score Far from Station (Tag 13)",
						selectIfConfident(
								sequence(score(13, 4, 21, 22, 17))));
		m_testingChooser
				.addOption(
						"Left Align to the Closest Tag",
						toClosestTag(kRobotToTagsLeft));
		m_testingChooser
				.addOption(
						"Right Align to the Closest Tag",
						toClosestTag(kRobotToTagsRight));
		m_testingChooser
				.addOption(
						"Check All Subsystems",
						parallel(
								sequence(
										m_elevatorSubsystem.testCommand(2.0),
										parallel(
												m_cheeseStickSubsystem.testCommand(2.0),
												m_wristSubsystem.testCommand(2.0))),
								m_driveSubsystem.testCommand(0.5, Math.toRadians(45), 1.0)));
		m_testingChooser
				.addOption(
						"Check CheeseStickSubsystem",
						m_cheeseStickSubsystem.testCommand(2.0));
		m_testingChooser
				.addOption(
						"Check WristSubsystem", m_wristSubsystem.testCommand(2.0));
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
		m_testingChooser
				.addOption(
						"Check PID Constants for Driving (5'x5' Square)",
						moveOnSquare(feetToMeters(5), distanceTolerance, angleToleranceInDegrees, 16));
		m_testingChooser
				.addOption(
						"Check kWheelDiameter (F/B 6 feet)",
						moveForwardBackward(feetToMeters(6), distanceTolerance, angleToleranceInDegrees));
		m_testingChooser
				.addOption(
						"Slowest Movement Test (F/B/L/R/LR/RR and F/B while rotating)",
						m_driveSubsystem.testCommand(kDriveMinSpeed, kTurnMinAngularSpeed, 1.0));
	}

	public void bindAlert(Alert alert, BooleanSupplier event) {
		CommandScheduler.getInstance().getActiveButtonLoop().bind(() -> alert.set(event.getAsBoolean()));
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
						() -> -m_driverController.getRightY(),
						() -> -m_driverController.getRightX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getCreateButton)); // makes the robot
		// robot-oriented

		/// TODO: button binding needed with the correct button
		m_driverController.L1().whileTrue(
				toClosestTag(kRobotToTagsLeft).withName("toClosestTag(kRobotToTagsLeft)"));
		m_driverController.R1().whileTrue(
				toClosestTag(kRobotToTagsRight).withName("toClosestTag(kRobotToTagsRight)"));
		m_driverController.PS().onTrue(toClosestTag(kRobotToTags).withName("toClosestTag(kRobotToTagMiddle)"));
		m_driverController.options().onTrue(m_driveSubsystem.resetHeading());
		m_driverController.square().onTrue(m_driveSubsystem.toggleCoastMode());
	}

	public void bindElevatorControls() {
		RobotModeTriggers.disabled().onTrue(m_elevatorSubsystem.stopMotor());
		m_operatorController.axisMagnitudeGreaterThan(PS5Controller.Axis.kLeftY.value, kDeadzone)
				.whileTrue(m_elevatorSubsystem.manualMove(() -> -m_operatorController.getLeftY()));
		m_operatorController.triangle().onTrue(
				prepareToScore(4, false)
						.withName("Elevator to Level Four and Wrist to Angle"));
		m_operatorController.square().onTrue(
				prepareToScore(3, false)
						.withName("Elevator to Level Three and Wrist to Angle"));
		m_operatorController.cross().onTrue(
				prepareToScore(2, false)
						.withName("Elevator to Level Two and Wrist to Angle"));
		m_operatorController.circle().onTrue(scoreLevelOneInTeleop());
		m_operatorController.L1().and(m_operatorController.triangle()).onTrue(removeAlgaeLevelThree());
		m_operatorController.L1().and(m_operatorController.square()).onTrue(removeAlgaeLevelTwo());
		m_operatorController.L1().and(m_operatorController.circle()).onTrue(prepareForCoralPickup());
		m_operatorController.L1().and(m_operatorController.cross()).onTrue(goToBase());
		m_operatorController.touchpad().onTrue(m_elevatorSubsystem.stopMotor());
		m_operatorController.create().onTrue(m_elevatorSubsystem.resetTheEncoder());
	}

	public void bindAlgaeControls() {
		// m_algaeGrabberSubsystem
		// .setDefaultCommand(m_algaeGrabberSubsystem.manualMove(() ->
		// m_operatorController.getRightX()));
		m_operatorController.L2().onTrue(
				m_algaeGrabberSubsystem.grabAlgaeAndHold()
						.andThen(m_arduinoSubsystem.ledPattern(StatusCode.INTAKED_ALGAE))
						.withName("Grab Algae and Hold"));
		m_operatorController.R2().onTrue(m_algaeGrabberSubsystem.releaseAlgae());
	}

	public void bindWristControls() {
		m_wristSubsystem.setDefaultCommand(m_wristSubsystem.manualMove(() -> m_operatorController.getRightY()));
	}

	public void bindCheeseStickControls() {
		m_operatorController.R1().whileFalse(m_cheeseStickSubsystem.grab());
		m_operatorController.R1().whileTrue(m_cheeseStickSubsystem.release());

		m_driverController.circle().whileFalse(m_cheeseStickSubsystem.grab());
		m_driverController.circle().whileTrue(m_cheeseStickSubsystem.release());
	}

	public void bindClimberControls() {
		// m_climberSubsystem.setDefaultCommand(m_climberSubsystem.manualMove(() ->
		// m_driverController.getRightY()));
		m_driverController.triangle().onTrue(m_climberSubsystem.deploy());
		m_driverController.cross().onTrue(retractClimber());

		m_operatorController.povUp().onTrue(retractClimber());
		m_operatorController.povDown().onTrue(m_climberSubsystem.deploy());
	}

	public void bindLEDControls() {
		m_operatorController.povRight().onTrue(m_arduinoSubsystem.ledPattern(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_operatorController.povLeft().onTrue(m_arduinoSubsystem.ledPattern(StatusCode.DEFAULT));
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
		m_cheeseStickSubsystem.grab();
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
