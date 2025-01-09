// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.Arrays;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.SwerveModule;

/**
 * Refer to the proper documentation here:
 * https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
 * https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
 * 
 * FROM 2 YEARS AGO: https://www.youtube.com/watch?v=0Xi9yb1IMyA
 * 
 * @author Alex Chan
 * @author Natalie Mann
 * 
 */

public class DriveSubsystem extends SubsystemBase {
	/** Swerve Modules **/
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;
	/** Kinematics declaration, including mod locations and the odometry object **/
	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kFrontLeftLocation,
			kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDriveOdometry m_odometry;
	/** NavX declaration (our gyro, controls the heading) **/
	private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
	/**
	 * Heading --> Think circle rotation, this is degrees of it, or the heading
	 * (like a plane)
	 **/
	private double m_headingOffset = 0;
	private Rotation2d m_heading = new Rotation2d();
	/**
	 * SysID: " The routine will generate control signals which user-defined
	 * callbacks will send to the motors being characterized, while the robot
	 * records data into a log file."
	 **/
	private final SysIdRoutine m_sysidRoutine;

	/** Published things (network tables) **/
	private final ProtobufPublisher<Pose2d> m_posePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher; // TODO: Have Alex explain these
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;

	/** Creates a new DriveSubsystem. **/
	public DriveSubsystem() {
		// TODO
		var config = new SysIdRoutine.Config(Units.Volts.of(2.5).div(Units.Seconds.of(1)), null, Units.Seconds.of(3));

		m_sysidRoutine = new SysIdRoutine(config, new SysIdRoutine.Mechanism((volt) -> {
			SwerveModuleState[] pos = new SwerveModuleState[4];
			Arrays.fill(pos, new SwerveModuleState(volt.magnitude(), new Rotation2d(Math.PI / 2)));
			setModuleStates(pos);
		}, null, this));

		/** Publish Network Tables **/
		m_posePublisher = NetworkTableInstance.getDefault().getProtobufTopic("/SmartDashboard/Pose", Pose2d.proto)
				.publish();
		m_targetModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_currentModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleState.struct)
				.publish();

		/** Make every Swerve module actually have something in it **/
		m_frontLeft = new SwerveModule(kFrontLeftCANCoderPort, kFrontLeftDrivePort, kFrontLeftSteerPort);
		m_frontRight = new SwerveModule(kFrontRightCANCoderPort, kFrontRightDrivePort, kFrontRightSteerPort);
		m_backLeft = new SwerveModule(kBackLeftCANCoderPort, kBackLeftDrivePort, kBackLeftSteerPort);
		m_backRight = new SwerveModule(kBackRightCANCoderPort, kBackRightDrivePort, kBackRightSteerPort);

		m_gyro.zeroYaw();
		resetEncoders();

		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
	}

	/**
	 * Sets a gyro offset.
	 * 
	 * @param angle The angle to offset gyro readings (CCW+).
	 */
	public void setHeadingOffset(double angle) {
		m_headingOffset = angle;
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public Rotation2d getHeading() {
		if (RobotBase.isSimulation()) {
			return m_heading;
		}
		return Rotation2d.fromDegrees(-m_gyro.getYaw() + m_headingOffset);
	}

	/**
	 * Resets gyro heading to zero.
	 */
	public void resetHeading() {
		m_gyro.reset();
	}

	/**
	 * Resets drive encoders to zero.
	 */
	public void resetEncoders() {
		// Zero drive encoders
		m_frontLeft.resetDriveEncoder();
		m_frontRight.resetDriveEncoder();
		m_backLeft.resetDriveEncoder();
		m_backRight.resetDriveEncoder();
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
	 * Calculates the modules states needed for the robot to achieve the target
	 * chassis speed.
	 * 
	 * @param speeds The target chassis speed
	 * @param isFieldRelative Whether or not the chassis speeds are field-relative
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	public SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds, boolean isFieldRelative) {
		if (isFieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}
		SmartDashboard.putNumber("Heading Radians", getHeading().getRadians());
		SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
		return states;
	}

	/**
	 * 
	 * Stops all the motors.
	 */
	public void stopDriving() {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(0, 0, 0), true));
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
	 * Sets module states for each swerve module.
	 * 
	 * @param moduleStates The module states, in order of FL, FR, BL, BR
	 */
	public void setModuleStates(SwerveModuleState[] moduleStates) {
		// Get the current module angles
		double[] moduleAngles = { m_frontLeft.getModuleAngle(), m_frontRight.getModuleAngle(),
				m_backLeft.getModuleAngle(), m_backRight.getModuleAngle() };
		for (int i = 0; i < moduleStates.length; i++) {
			// Optimize target module states
			moduleStates[i].optimize(Rotation2d.fromDegrees(moduleAngles[i]));

		}
		m_targetModuleStatePublisher.set(moduleStates);

		m_frontLeft.setModuleState(moduleStates[0]);
		m_frontRight.setModuleState(moduleStates[1]);
		m_backLeft.setModuleState(moduleStates[2]);
		m_backRight.setModuleState(moduleStates[3]);
	}

	/**
	 * Directly sets the module angle. Do not use for general driving.
	 * 
	 * @param angle The angle in degrees
	 */
	public void setModuleAngles(double angle) {
		m_frontLeft.setAngle(angle);
		m_frontRight.setAngle(angle);
		m_backLeft.setAngle(angle);
		m_backRight.setAngle(angle);
	}

	public void setModuleStatesDirect(SwerveModuleState moduleState) {
		m_frontLeft.setModuleState(moduleState);
		m_frontRight.setModuleState(moduleState);
		m_backLeft.setModuleState(moduleState);
		m_backRight.setModuleState(moduleState);
	}

	/**
	 * Sets module states for each swerve module.
	 * 
	 * @param speedFwd The forward speed in voltage
	 * @param speedSide The sideways speed in voltage
	 * @param speedRot The rotation speed in voltage
	 * @param isFieldRelative Whether or not the speeds are relative to the field
	 */
	public void setModuleStates(double speedFwd, double speedSide, double speedRot, boolean isFieldRelative) {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(speedFwd, speedSide, speedRot), isFieldRelative));
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Current Position",
		// getModulePositions()[0].distanceMeters);
		// SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
		SwerveModuleState[] states = { m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
				m_backLeft.getModuleState(), m_backRight.getModuleState() };
		m_currentModuleStatePublisher.set(states);
	}

	/**
	 * Creates a command to drive the robot with joystick input.
	 *
	 * @return A command to drive the robot.
	 */
	public Command driveCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Double> rotationRight, Supplier<Double> rotationLeft) {
		return run(() -> {
			// Get the forward, strafe, and rotation speed, using a deadband on the joystick
			// input so slight movements don't move the robot
			double rotSpeed = MathUtil.applyDeadband(
					(rotationRight.get() - rotationLeft.get()),
					ControllerConstants.kDeadzone);
			rotSpeed = -Math.signum(rotSpeed) * Math.pow(rotSpeed, 2) * kTeleopMaxTurnVoltage;

			double fwdSpeed = MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = -Math.signum(fwdSpeed) * Math.pow(fwdSpeed, 2) * kTeleopMaxVoltage;

			double strSpeed = MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = -Math.signum(strSpeed) * Math.pow(strSpeed, 2) * kTeleopMaxVoltage;

			setModuleStates(fwdSpeed, strSpeed, rotSpeed, true);
		}).withName("DefaultDriveCommand");
	}

	/**
	 * Creates a command to drive the robot with joystick input in robot oriented
	 * controls.
	 * 
	 * @return A command to drive the robot.
	 */
	public Command robotOrientedDriveCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Double> rotationRight, Supplier<Double> rotationLeft) {
		return run(() -> {
			// Get the forward, strafe, and rotation speed, using a deadband on the joystick
			// input so slight movements don't move the robot
			double rotSpeed = kTeleopMaxTurnVoltage * MathUtil.applyDeadband(
					(rotationRight.get() - rotationLeft.get()),
					ControllerConstants.kDeadzone);
			rotSpeed = Math.signum(rotSpeed) * (rotSpeed * rotSpeed) * 12.0;
			double fwdSpeed = kTeleopMaxVoltage
					* MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = -Math.signum(fwdSpeed) * (fwdSpeed * fwdSpeed) * 12.0;
			double strSpeed = kTeleopMaxVoltage
					* MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = -Math.signum(strSpeed) * (strSpeed * strSpeed) * 12.0;
			setModuleStates(fwdSpeed, strSpeed, rotSpeed, false);
		}).withName("RobotOrientedDriveCommand");
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeadingCommand() {
		return runOnce(m_gyro::zeroYaw);
	}

	public Command offsetGyroCommand(double angle) {
		return runOnce(() -> setHeadingOffset(angle));
	}

	/**
	 * Creates a command to reset the drive encoders to zero.
	 * 
	 * @return A command to reset the drive encoders.
	 */
	public Command resetEncodersCommand() {
		return runOnce(() -> {
			resetEncoders();
			m_odometry.resetPosition(getHeading(), getModulePositions(), new Pose2d());
		});
	}

	/**
	 * Creates a command to align the swerve modules to zero degrees relative to the
	 * robot.
	 * 
	 * @return A command to align the swerve modules.
	 */
	public Command alignModulesToZeroComamnd() {
		return run(() -> {
			m_kinematics.resetHeadings(
					new Rotation2d[] { new Rotation2d(0), new Rotation2d(0), new Rotation2d(0), new Rotation2d(0) });
			setModuleStates(0, 0, 0, false);
		}).raceWith(Commands.waitSeconds(5));
	}

	/**
	 * Creates a command that drives forward for a specified time period.
	 * 
	 * @param seconds The amount of time to drive for.
	 * @return The command.
	 */
	public Command driveForTimeCommand(double seconds) {
		return runEnd(() -> setModuleStates(12, 0, 0, true), this::stopDriving).withTimeout(seconds);
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
}
