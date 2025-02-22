// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.CommandComposer.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.List;
import java.util.Map;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.AlgaeGrabberSubsystem.GrabberState;
import frc.robot.subsystems.CheeseStickSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSimulator;
import frc.robot.subsystems.WristSubsystem;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private Command m_testCommand;
	private final SendableChooser<Command> m_testSelector = new SendableChooser<Command>();
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
	private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
	private final VisionSimulator m_visionSimulator = new VisionSimulator(m_driveSubsystem,
			pose(kFieldLayout.getFieldLength() / 2 + 2.5, 1.91 + .3, 180), 0.01);
	SimCameraProperties cameraProp = new SimCameraProperties() {
		{
			setCalibration(640, 480, Rotation2d.fromDegrees(100));
			// Approximate detection noise with average and standard deviation error in
			// pixels.
			setCalibError(0.25, 0.08);
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
		var dropChute = new MechanismLigament2d("bottom", Units.inchesToMeters(5), 0, 5, new Color8Bit(Color.kBeige));
		dropChute.append(new MechanismLigament2d("side", Units.inchesToMeters(12), 90, 5, new Color8Bit(Color.kWhite)));
		m_mechanism.getRoot("dropChute", Units.inchesToMeters(28), Units.inchesToMeters(9)).append(dropChute);
		SmartDashboard.putData("Superstructure", m_mechanism);

		double distanceTolerance = 0.01;
		double angleToleranceInDegrees = 1.0;
		double intermediateDistanceTolerance = 0.16;
		double intermediateAngleToleranceInDegrees = 16.0;
		m_testSelector
				.addOption(
						"Quickly Align to AprilTags 1, 2, 6, 7, and 8",
						CommandComposer.alignToTags(
								distanceTolerance, angleToleranceInDegrees, intermediateDistanceTolerance,
								intermediateAngleToleranceInDegrees,
								List.of(transform(1.5, 0, 180), transform(1.0, 0, 180), transform(.5, 0, 180)),
								transform(1.5, 0, 180), 7, 6, 1,
								6, 7, 8, 2, 8, 7));
		m_testSelector
				.addOption(
						"Check kDriveGearRatio and kWheelDiameter (F/B 6 feet)",
						CommandComposer.moveForwardBackward(6, distanceTolerance, angleToleranceInDegrees));
		m_testSelector
				.addOption(
						"Check PID Constants for Driving (5'x5' Square)",
						CommandComposer
								.moveOnSquare(Units.feetToMeters(5), distanceTolerance, angleToleranceInDegrees, 16));
		m_testSelector.addOption("Test DriveSubsystem", m_driveSubsystem.testCommand());
		SmartDashboard.putData("Test Selector", m_testSelector);

		SmartDashboard.putData(m_pdh);

		SmartDashboard.putData(CommandScheduler.getInstance());
		DataLogManager.start();
		DataLogManager.logNetworkTables(true);
		URCL.start(
				Map.of(
						11, "FR Turn", 21, "BR Turn", 31, "BL Turn", 41, "FL Turn", kElevatorMotorPort, "Elevator",
						kClimberMotorPort, "Climber Motor", kWristMotorPort, "Wrist Motor", kFlywheelMotorPort,
						"Algae Motor"));
		DriverStation.startDataLog(DataLogManager.getLog());
		bindDriveControls();
		bindElevatorControls();
		bindWristControls();
		bindAlgaeControls();
	}

	public void bindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_driverController.getLeftY(),
						() -> -m_driverController.getLeftX(),
						() -> -m_driverController.getRightY(),
						() -> m_driverController.getRightX(),
						m_driverController.getHID()::getSquareButton)); // makes the robot robot-oriented
		m_driverController.options().onTrue(m_driveSubsystem.resetHeading());
	}

	public void bindElevatorControls() {
		m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.manualMove(() -> -m_operatorController.getLeftY()));
		m_operatorController.triangle().onTrue(scoreLevelFour());
		m_operatorController.square().onTrue(scoreLevelThree());
		m_operatorController.cross().onTrue(scoreLevelTwo());
		m_operatorController.circle().onTrue(scoreLevelOne());
		// m_operatorController.povLeft().onTrue(m_elevatorSubsystem.goToCoralStationHeight());
		m_operatorController.L1().and(m_operatorController.circle()).onTrue(m_elevatorSubsystem.goToBaseHeight());
	}

	public void bindAlgaeControls() {
		m_operatorController.L2().onTrue(
				m_algaeGrabberSubsystem.deployGrabber(GrabberState.DOWN)
						.andThen(m_algaeGrabberSubsystem.runFlywheel()).until(
								() -> m_algaeGrabberSubsystem.checkCurrentOnFlywheel())
						.andThen(m_algaeGrabberSubsystem.slowRunFlywheel()));
		m_operatorController.R2().onTrue(
				m_algaeGrabberSubsystem.deployGrabber(GrabberState.UP)
						.andThen(m_algaeGrabberSubsystem.stopFlywheel()));

		m_operatorController.options().onTrue(m_algaeGrabberSubsystem.runFlywheelReverse());
		m_operatorController.options().onFalse(m_algaeGrabberSubsystem.stopFlywheel());
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
		m_operatorController.povDown().whileTrue(m_climberSubsystem.moveForward())
				.onFalse(m_climberSubsystem.moveBackward());
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
		m_autonomousCommand = null;

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
