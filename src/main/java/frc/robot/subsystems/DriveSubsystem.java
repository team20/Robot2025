// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

public class DriveSubsystem extends SubsystemBase {
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDriveOdometry m_odometry;
	private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
	private final SimDouble m_gyroSim;
	// https://docs.wpilib.org/en/latest/docs/software/advanced-controls/system-identification/index.html
	private final SysIdRoutine m_sysidRoutine;

	private final StructPublisher<Pose2d> m_posePublisher;
	private final StructPublisher<ChassisSpeeds> m_currentChassisSpeedsPublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;
	private final StructPublisher<Rotation2d> m_targetHeadingPublisher;

	private final PIDController m_orientationController = new PIDController(kRotationP, kRotationI, kRotationD);

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		m_orientationController.enableContinuousInput(-Math.PI, Math.PI);
		m_posePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Pose", Pose2d.struct)
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
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
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
		return m_odometry.getPoseMeters();
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
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
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
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeading() {
		return runOnce(m_gyro::zeroYaw).withName("ResetHeadingCommand");
	}

	public Command resetOdometry(Pose2d pose) {
		return runOnce(() -> m_odometry.resetPosition(getHeading(), getModulePositions(), pose))
				.withName("ResetOdometryCommand");
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

	/**
	 * Creates a {@code Command} for testing this {@code DriveSubsystem}. The robot
	 * must move forward, backward, strafe left, strafe right, turn left, turn
	 * right, and moving forward and backward while turning.
	 * 
	 * @param speed the speed in meters per second
	 * @param rotionalSpeed the angular speed in radians per second
	 * @param duration the duration of each movement in seconds
	 * 
	 * @return a {@code Command} for testing this {@code DriveSubsystem}
	 */
	public Command testCommand(double speed, double rotionalSpeed, double duration) {
		return sequence(
				run(() -> drive(speed, 0, 0, false)).withTimeout(duration),
				run(() -> drive(-speed, 0, 0, false)).withTimeout(duration),
				run(() -> drive(0, speed, 0, false)).withTimeout(duration),
				run(() -> drive(0, -speed, 0, false)).withTimeout(duration),
				run(() -> drive(0, 0, rotionalSpeed, false)).withTimeout(duration),
				run(() -> drive(0, 0, -rotionalSpeed, false)).withTimeout(duration),
				run(() -> drive(speed, 0, rotionalSpeed, true)).withTimeout(duration),
				run(() -> drive(-speed, 0, -rotionalSpeed, true)).withTimeout(duration),
				run(() -> drive(0, 0, 0, true)));
	}

}
