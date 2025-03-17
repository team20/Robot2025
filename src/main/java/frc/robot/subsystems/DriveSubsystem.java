// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.vision.VisionSubsystem.VisionConsumer;

public class DriveSubsystem extends SubsystemBase implements VisionConsumer {
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;
	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDrivePoseEstimator m_poseEstimator;
	private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);
	private final SimDouble m_gyroSim;
	// https://docs.wpilib.org/en/latest/docs/software/advanced-controls/system-identification/index.html
	private final SysIdRoutine m_sysidRoutine;
	private final StructPublisher<Pose2d> m_posePublisher;
	private final StructPublisher<Pose2d> m_visionPosePublisher;
	private final StructPublisher<ChassisSpeeds> m_currentChassisSpeedsPublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;
	private final StructPublisher<Rotation2d> m_targetHeadingPublisher;
	private final PIDController m_orientationController = new PIDController(kRotationP, kRotationI, kRotationD);

	public DriveSubsystem() {
		m_orientationController.enableContinuousInput(-Math.PI, Math.PI);

		m_posePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Pose", Pose2d.struct)
				.publish();

		m_visionPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/VisionPose", Pose2d.struct)
				.publish();

		m_currentChassisSpeedsPublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Chassis Speeds", ChassisSpeeds.struct)
				.publish();
		m_targetModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_currentModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_targetHeadingPublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Target Heading", Rotation2d.struct)
				.publish();
		m_frontLeft = new SwerveModule(kFrontLeftCANCoderPort, kFrontLeftDrivePort, kFrontLeftSteerPort);
		m_frontRight = new SwerveModule(kFrontRightCANCoderPort, kFrontRightDrivePort, kFrontRightSteerPort);
		m_backLeft = new SwerveModule(kBackLeftCANCoderPort, kBackLeftDrivePort, kBackLeftSteerPort);
		m_backRight = new SwerveModule(kBackRightCANCoderPort, kBackRightDrivePort, kBackRightSteerPort);
		// Adjust ramp rate, step voltage, and timeout to make sure robot doesn't
		// collide with anything
		var config = new SysIdRoutine.Config(Volts.of(2.5).div(Seconds.of(1)), null, Seconds.of(3));
		m_sysidRoutine = new SysIdRoutine(config, new SysIdRoutine.Mechanism(volt -> {
			var state = new SwerveModuleState(volt.magnitude(), new Rotation2d(Math.PI / 2));
			m_frontLeft.setModuleState(state);
			m_frontRight.setModuleState(state);
			m_backLeft.setModuleState(state);
			m_backRight.setModuleState(state);
		}, null, this));
		m_gyro.zeroYaw();
		resetEncoders();
		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		// Initialize pose estimator with kinematics, initial gyro angle, initial module
		// positions,
		// and an initial pose (0,0,0)
		m_poseEstimator = new SwerveDrivePoseEstimator(
				m_kinematics,
				getHeading(),
				getModulePositions(),
				new Pose2d());

		if (RobotBase.isSimulation()) {
			m_gyroSim = new SimDeviceSim("navX-Sensor", m_gyro.getPort()).getDouble("Yaw");
		} else {
			m_gyroSim = null;
		}
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public Rotation2d getHeading() {
		return m_gyro.getRotation2d();
	}

	/**
	 * Resets drive encoders to zero.
	 */
	private void resetEncoders() {
		m_frontLeft.resetDriveEncoder();
		m_frontRight.resetDriveEncoder();
		m_backLeft.resetDriveEncoder();
		m_backRight.resetDriveEncoder();
	}

	/**
	 * Returns the {@code SwerveDriveKinematics} used by this
	 * {@code DriveSubsystem}.
	 * 
	 * @return the {@code SwerveDriveKinematics} used by this {@code DriveSubsystem}
	 */
	public SwerveDriveKinematics kinematics() {
		return m_kinematics;
	}

	/**
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	/**
	 * Gets the module positions for each swerve module.
	 * 
	 * @return The module positions, in order of FL, FR, BL, BR
	 */
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] { m_frontLeft.getModulePosition(), m_frontRight.getModulePosition(),
				m_backLeft.getModulePosition(), m_backRight.getModulePosition() };
	}

	/**
	 * Calculates module states from a chassis speeds.
	 * 
	 * @param speeds The chassis speeds.
	 * @param isFieldRelative Whether or not the chassis speeds is field relative.
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	private SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds, boolean isFieldRelative) {
		if (isFieldRelative)
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		speeds = ChassisSpeeds.discretize(speeds, 0.03);
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopDriveMaxSpeed);
		double[] moduleAngles = { m_frontLeft.getModuleAngle(), m_frontRight.getModuleAngle(),
				m_backLeft.getModuleAngle(), m_backRight.getModuleAngle() };
		for (int i = 0; i < states.length; i++) // Optimize target module states
			states[i].optimize(Rotation2d.fromDegrees(moduleAngles[i]));
		return states;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speeds The chassis speeds.
	 */
	private void setModuleStates(SwerveModuleState[] states) {
		m_targetModuleStatePublisher.set(states);
		m_frontLeft.setModuleState(states[0]);
		m_frontRight.setModuleState(states[1]);
		m_backLeft.setModuleState(states[2]);
		m_backRight.setModuleState(states[3]);
	}

	/**
	 * Drives the robot.
	 * 
	 * @param vxMetersPerSecond the forward velocity in meters per second
	 * @param vyMetersPerSecond the sideways velocity in meters per second
	 * @param omegaRadiansPerSecond the angular velocity in radians per second
	 * @param isFieldRelative a boolean value indicating whether or not the
	 *        velocities are relative to the field
	 */
	public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
			boolean isFieldRelative) {
		setModuleStates(
				calculateModuleStates(
						chassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond),
						isFieldRelative));
	}

	/**
	 * Drives the robot.
	 * 
	 * @param chassisSpeeds the {@code ChassisSpeeds} for the robot
	 * @param isFieldRelative a boolean value indicating whether or not the
	 *        velocities are relative to the field
	 */
	public void drive(ChassisSpeeds chassisSpeeds, boolean isFieldRelative) {
		setModuleStates(calculateModuleStates(chassisSpeeds, isFieldRelative));
	}

	public void setDriveMotorNeutralMode(NeutralModeValue mode) {
		m_frontLeft.setNeutralMode(mode);
		m_frontRight.setNeutralMode(mode);
		m_backLeft.setNeutralMode(mode);
		m_backRight.setNeutralMode(mode);
	}

	/**
	 * Creates a {@code ChassisSpeeds} instance to drive the robot with joystick
	 * input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param forwardOrientation Forward orientation supplier. Positive values make
	 *        the robot face forward (+X direction).
	 * @param strafeOrientation Strafe orientation supplier. Positive values make
	 *        the robot face left (+Y direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public ChassisSpeeds chassisSpeeds(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier forwardOrientation, DoubleSupplier strafeOrientation, DoubleSupplier rotation) {
		var orientation = new Translation2d(forwardOrientation.getAsDouble(), strafeOrientation.getAsDouble());
		double omegaRadiansPerSecond = MathUtil.applyDeadband(rotation.getAsDouble(), ControllerConstants.kDeadzone);
		omegaRadiansPerSecond = Math.signum(omegaRadiansPerSecond) * Math.pow(omegaRadiansPerSecond, 2)
				* kTeleopTurnMaxAngularSpeed;
		if (orientation.getNorm() > 0.05) {
			var angle = orientation.getAngle();
			omegaRadiansPerSecond += m_orientationController
					.calculate(getHeading().getRadians(), angle.getRadians());
			m_targetHeadingPublisher.set(angle);
		}
		return chassisSpeeds(forwardSpeed, strafeSpeed, omegaRadiansPerSecond);
	}

	/**
	 * Creates a {@code ChassisSpeeds} instance to drive the robot with joystick
	 * input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param forwardOrientation Forward orientation supplier. Positive values make
	 *        the robot face forward (+X direction).
	 * @param strafeOrientation Strafe orientation supplier. Positive values make
	 *        the robot face left (+Y direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public ChassisSpeeds chassisSpeeds(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier forwardOrientation, DoubleSupplier strafeOrientation) {
		var orientation = new Translation2d(forwardOrientation.getAsDouble(), strafeOrientation.getAsDouble());
		double omegaRadiansPerSecond = 0;
		if (orientation.getNorm() > 0.05) {
			var angle = orientation.getAngle();
			omegaRadiansPerSecond = m_orientationController
					.calculate(getHeading().getRadians(), angle.getRadians());
			m_targetHeadingPublisher.set(angle);
		}
		return chassisSpeeds(forwardSpeed, strafeSpeed, omegaRadiansPerSecond);
	}

	/**
	 * Creates a {@code ChassisSpeeds} instance to drive the robot with joystick
	 * input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation Rotation supplier. Positive values make
	 *        the robot rotate left (CCW direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public static ChassisSpeeds chassisSpeeds(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation) {
		double omegaRadiansPerSecond = MathUtil.applyDeadband(rotation.getAsDouble(), ControllerConstants.kDeadzone);
		omegaRadiansPerSecond = Math.signum(omegaRadiansPerSecond) * Math.pow(omegaRadiansPerSecond, 2)
				* kTeleopTurnMaxAngularSpeed;
		return chassisSpeeds(forwardSpeed, strafeSpeed, omegaRadiansPerSecond);
	}

	/**
	 * Creates a {@code ChassisSpeeds} instance to drive the robot with joystick
	 * input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param omegaRadiansPerSecond angular velocity in radians per second
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	static ChassisSpeeds chassisSpeeds(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			double omegaRadiansPerSecond) {
		double vxMetersPerSecond = MathUtil.applyDeadband(forwardSpeed.getAsDouble(), ControllerConstants.kDeadzone);
		vxMetersPerSecond = Math.signum(vxMetersPerSecond) * Math.pow(vxMetersPerSecond, 2) * kTeleopDriveMaxSpeed;

		double vyMetersPerSecond = MathUtil.applyDeadband(strafeSpeed.getAsDouble(), ControllerConstants.kDeadzone);
		vyMetersPerSecond = Math.signum(vyMetersPerSecond) * Math.pow(vyMetersPerSecond, 2) * kTeleopDriveMaxSpeed;

		return chassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
	}

	/**
	 * Constructs a {@code ChassisSpeeds} object.
	 *
	 * @param vxMetersPerSecond forward velocity in meters per second
	 * @param vyMetersPerSecond sideways velocity in meters per second
	 * @param omegaRadiansPerSecond angular velocity in radians per second
	 */
	public static ChassisSpeeds chassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond,
			double omegaRadiansPerSecond) {
		vxMetersPerSecond = MathUtil.clamp(vxMetersPerSecond, -kTeleopDriveMaxSpeed, kTeleopDriveMaxSpeed);
		vyMetersPerSecond = MathUtil.clamp(vyMetersPerSecond, -kTeleopDriveMaxSpeed, kTeleopDriveMaxSpeed);
		omegaRadiansPerSecond = MathUtil
				.clamp(omegaRadiansPerSecond, -kTeleopTurnMaxAngularSpeed, kTeleopTurnMaxAngularSpeed);
		return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}. Useful
	 * for updating subsystem-specific state.
	 */
	@Override
	public void periodic() {
		SwerveModuleState[] states = { m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
				m_backLeft.getModuleState(), m_backRight.getModuleState() };
		m_currentModuleStatePublisher.set(states);
		var speeds = m_kinematics.toChassisSpeeds(states);
		m_currentChassisSpeedsPublisher.set(speeds);
		if (RobotBase.isSimulation())// TODO: Use SysId to get feedforward model for rotation
			m_gyroSim.set(-Math.toDegrees(speeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod) + m_gyro.getYaw());

		// Update pose estimator with current heading and module positions
		Pose2d estimatedPose = m_poseEstimator.update(getHeading(), getModulePositions());
		m_posePublisher.set(estimatedPose);
	}

	/**
	 * If robot is on brake changes to coast, else goes to brake
	 * 
	 * @return the command
	 */
	public Command toggleCoastMode() {
		AtomicBoolean shouldBeCoast = new AtomicBoolean(true);
		return runOnce(() -> {
			NeutralModeValue mode;
			if (shouldBeCoast.get()) {
				mode = NeutralModeValue.Coast;
			} else {
				mode = NeutralModeValue.Brake;
			}
			shouldBeCoast.set(!shouldBeCoast.get());
			setDriveMotorNeutralMode(mode);
		}).withName("Drive Toggle Coast Mode");
	}

	/**
	 * Command to set the robot to coast
	 * 
	 * @param mode what mode to set the motors to
	 * @return the command
	 */
	public Command setNeutralMode(NeutralModeValue mode) {
		return runOnce(() -> setDriveMotorNeutralMode(mode)).withName("Drive Enable Coast Mode");
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeading() {
		return runOnce(m_gyro::zeroYaw).withName("ResetHeadingCommand");
	}

	/**
	 * Command of {@link edu.wpi.first.math.kinematics.Odometry#resetPosition()}
	 * 
	 * @param pose the position that the robot is at on the field
	 * @return runs the command once
	 */
	public Command resetOdometry(Pose2d pose) {
		return runOnce(() -> m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose))
				.withName("ResetOdometryCommand");
	}

	/**
	 * Creates a {@code Command} to drive the robot with joystick input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param forwardOrientation Forward orientation supplier. Positive values make
	 *        the robot face forward (+X direction).
	 * @param strafeOrientation Strafe orientation supplier. Positive values make
	 *        the robot face left (+Y direction).
	 * @param isRobotRelative Supplier for determining if driving should be robot
	 *        relative.
	 * @return A command to drive the robot.
	 */
	public Command driveCommand(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier forwardOrientation, DoubleSupplier strafeOrientation, DoubleSupplier rotation,
			BooleanSupplier isRobotRelative) {
		return run(
				() -> drive(
						chassisSpeeds(forwardSpeed, strafeSpeed, forwardOrientation, strafeOrientation, rotation),
						!isRobotRelative.getAsBoolean())).withName("DefaultDriveCommand");
	}

	/**
	 * Creates a {@code Command} to drive the robot with joystick input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param forwardOrientation Forward orientation supplier. Positive values make
	 *        the robot face forward (+X direction).
	 * @param strafeOrientation Strafe orientation supplier. Positive values make
	 *        the robot face left (+Y direction).
	 * @param isRobotRelative Supplier for determining if driving should be robot
	 *        relative.
	 * @return A command to drive the robot.
	 */
	public Command driveCommand(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier forwardOrientation, DoubleSupplier strafeOrientation, BooleanSupplier isRobotRelative) {
		return run(
				() -> drive(
						chassisSpeeds(forwardSpeed, strafeSpeed, forwardOrientation, strafeOrientation),
						!isRobotRelative.getAsBoolean())).withName("DefaultDriveCommand");
	}

	/**
	 * Creates a {@code Command} to drive the robot with joystick input.
	 *
	 * @param forwardSpeed Forward speed supplier. Positive values make the robot
	 *        go forward (+X direction).
	 * @param strafeSpeed Strafe speed supplier. Positive values make the robot
	 *        go to the left (+Y direction).
	 * @param rotation Rotation supplier. Positive values make
	 *        the robot rotate left (CCW direction).
	 * @return a {@code ChassisSpeeds} instance to drive the robot with joystick
	 *         input
	 */
	public Command driveCommand(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed,
			DoubleSupplier rotation, BooleanSupplier isRobotRelative) {
		return run(() -> drive(chassisSpeeds(forwardSpeed, strafeSpeed, rotation), !isRobotRelative.getAsBoolean()))
				.withName("DefaultDriveCommand");
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidDynamic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.dynamic(direction);
	}

	@Override
	public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
		m_visionPosePublisher.accept(visionRobotPoseMeters);

		// Update the pose estimator with the vision measurement
		m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

}
