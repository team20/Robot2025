package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {
	public static final class AlgaeConstants {
		public static final int kFlywheelMotorPort = 23;
		public static final int kGrabberAnglePort = 24;
		public static final boolean kFlywheelInvert = false;
		public static final boolean kGrabberAngleInvert = false;

		public static final int kSmartCurrentLimit = 15;
		public static final double kSecondaryCurrentLimit = 20;
		public static final double kTimeOverCurrentToStop = .1;

		public static final double kDeployGrabberPosition = .75;
		public static final double kFlywheelSpeed = .8;

		public static final double kP = 0.7;
		public static final double kI = 0.0;
		public static final double kD = 0;

		public static final double kAlgaePivotForwardSoftLimit = 0;
		public static final double kAlgaePivotReverseSoftLimit = 0;
	}

	public static final class CheeseStickConstants {
		public static final int kServoPort = 0;
		public static final double kReleaseDistance = 0.4;
		/**
		 * Set this value to how far the cheese stick wheels extend beyond the lexan.
		 */
		public static final Distance kExtensionLength = Inch.of(.5);
	}

	public static final class ClimberConstants {
		public static final int kClimberMotorPort = 25;
		public static final double kClimberForwardSoftLimit = .5;
		public static final double kClimberReverseSoftLimit = 0;
		public static final double kSpeed = 0.75;
		public static final double kP = 0.75;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kTolerance = 1;
		public static final int kSmartCurrentLimit = 50;
		public static final int kSecondaryCurrentLimit = kSmartCurrentLimit + 15;
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
		public static final double kTriggerDeadzone = .05;
	}

	public static final class DriveConstants {
		// CAN IDs (updated)
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.09;
		public static final double kI = 0.0;
		public static final double kD = 0.001;
		public static final double kS = 0;
		public static final double kV = 0.12;
		public static final double kA = 0.009;

		public static final double kRotationP = 5;
		public static final double kRotationI = 0.0;
		public static final double kRotationD = 0.1;
		public static final double kRotationS = 0;
		public static final double kRotationV = 1.9;
		public static final double kRotationA = 0.009;

		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kDriveGearRatio = 6.75;
		public static final double kSteerGearRatio = 150.0 / 7;
		public static final double kWheelDiameter = Units.inchesToMeters(4) * 0.95;
		// TODO: Finalize: decrease if overestimates travel distance
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio;

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

		public static final int kEncoderDepth = 4;
		public static final int kEncoderMeasurementPeriod = 16;
		public static final int kSteerSmartCurrentLimit = 60;
		public static final int kSteerSecondaryCurrentLimit = kSteerSmartCurrentLimit + 15;
		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1;
		public static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration();
		static {
			kDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kDriveConfig.CurrentLimits.SupplyCurrentLimit = 45; // For avoiding brownout
			kDriveConfig.CurrentLimits.SupplyCurrentLowerLimit = 45;
			kDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kDriveConfig.CurrentLimits.StatorCurrentLimit = 80; // Output current (proportional to acceleration)
			kDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kDriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampRate;
			kDriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = kRampRate;
		}

		public static final double kTeleopDriveMaxSpeed = 12.0; // 12 meters per second
		public static final double kTeleopTurnMaxAngularSpeed = Math.toRadians(360 * 5);// 5 rotations per second

		public static final double kDriveMaxSpeed = 12.0; // 12 meters per second
		public static final double kDriveMinSpeed = 0.2; // 0.2 meters per second
		public static final double kTurnMaxAngularSpeed = Math.toRadians(360); // 1 rotation per second
		public static final double kTurnMinAngularSpeed = Math.toRadians(0); // 0 degree per second

		// DriveCommand.java Constants
		public static final double kDriveP = 5; // TODO: Optimize: up to 12?
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxAcceleration = 2 * kDriveMaxSpeed; // kDriveMaxSpeed in 0.5 sec
		// TODO: Optimize: up to 8?

		public static final double kTurnP = 5;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0.1;
		public static final double kTurnMaxAcceleration = 2 * kTurnMaxAngularSpeed; // kTurnMaxAngularSpeed in 0.5
	}

	public static final class ElevatorConstants {
		public static final int kElevatorMotorPort = 26;
		public static final int kSmartCurrentLimit = 60;
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kP = 6.0; // 1.1
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kS = 0.05631;
		public static final double kG = 0.43;
		public static final double kV = 5.3794;
		public static final double kA = 0.74041;
		public static final double kGearRatio = 10;
		/**
		 * 24 teeth, 5 mm pitch, one rotation moves 120 mm, 2 stage cascading elevator
		 * means total height change is 240 mm.
		 */
		public static final double kMetersPerPulleyRotation = (24.0 * 5 * 2 / 1000);
		/**
		 * <pre>
		 * 				   1 pulley rotation	 pulley circumference
		 * 1 motor rot * --------------------- * --------------------
		 *               kGearRatio motor rots    1 pulley rotation
		 * </pre>
		 */
		public static final double kMetersPerMotorRotation = (1 / kGearRatio) * kMetersPerPulleyRotation;
		public static final double kMaxVelocity = 2.75;
		public static final double kMaxAccel = 2.5;
		public static final double kTolerance = 0.01;
		public static final double kLevelOneHeight = Units.inchesToMeters(3);
		public static final double kLevelTwoHeight = Units.inchesToMeters(8);
		public static final double kLevelThreeHeight = Units.inchesToMeters(29);
		public static final double kLevelFourHeight = Units.inchesToMeters(48 + 2);
		public static final double kMaxExtension = Units.inchesToMeters(49.5 + 0.75);
		// The amount that the elevator decreases in order to score
		public static final double kClearanceHeight = Units.inchesToMeters(5.5);
		public static final double kToScoreHeightDecrease = Units.inchesToMeters(0);
		public static final double kCoralStationHeight = Units.inchesToMeters(17 + 2);

		public static final double kAlgaeLevelThreeHeight = Units.inchesToMeters(0.25);
		public static final double kAlgaeLevelTwoHeight = Units.inchesToMeters(14);
		// public static final double kAlgaeLevelTwoAutoHeight =
		// Units.inchesToMeters(13.5);
	}

	public static final class WristConstants {
		public static final int kWristMotorPort = 27;
		public static final int kSmartCurrentLimit = 20;
		public static final int kSecondaryCurrentLimit = 20;
		public static final int kGrabberAngleLevelFour = 223; // 228 with wrist offset
		public static final int kGrabberAngleOthers = 221;
		public static final int kGrabberAngleLevelThree = 240; // 232 with wrist offset
		public static final double kAlgaeWristHeight = 170;

		public static final double kWristForwardSoftLimit = 274; // Wrist facing down
		public static final double kWristReverseSoftLimit = 90; // Wrist facing up
		public static final double kWristOffset = 0.75; // 3.5/260 for offset

		public static final double kP = 0.01; // TODO: Finalize
		public static final double kI = 0.0;
		public static final double kD = 0.003; // TODO: Finalize

		public static final double kTolerance = 4;
	}

	public static final class AutoAlignConstants {
		/**
		 * The {@code AprilTagFieldLayout}.
		 */
		public static AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

		/**
		 * The {@code Transform3d} expressing the pose of the first camera relative to
		 * the pose of the robot.
		 */
		public static Transform3d kRobotToCamera1 = new Transform3d(new Translation3d(0.3, 0.0, 0.2),
				new Rotation3d(0, Units.degreesToRadians(-20), 0));
		// TODO: Finalize

		/**
		 * The {@code Transform3d} expressing the pose of the second camera relative to
		 * the pose of the robot.
		 */
		public static Transform3d kRobotToCamera2 = new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
				new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180)));
		// TODO: Finalize

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTags = { transform(1.1, 0.0, 180),
				transform(0.60, 0.0, 180) };
		// transform(0.35, 0.0, 180) };
		// TODO: Check

		static Transform2d[] kRobotToStationTags = { transform(1.1, 0.0, 180), transform(0.1, 0, 180) };

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the left of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsLeft = { transform(1.1, 0, 180),
				transform(0.60, -0.165, 180) };
		// transform(0.60, -0.165, 180) };
		// transform(0.45, -0.185, 180) };
		// TODO: Check: decrease y to align more to the left

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the right of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsRight = { transform(1.1, 0, 180),
				transform(0.60, 0.195, 180) };
		// transform(0.60, 0.195, 180) };
		// transform(0.54, 0.200, 180) };
		// TODO: Check: decrease y to align more to the left

		/**
		 * The offset for alignment to algaes (positive: closer to algae).
		 */
		static double kForwrdAdjustmentAlgaeRemoval = 0.25;
		// TODO: Check

		/**
		 * The offset for alignment to coral stations (positive: closer to station).
		 */
		static double kForwrdAdjustmentCoralStation = 0.1;
		// TODO: Check

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the left of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsLeftReady = { transform(1.1, -0.5, 180),
				transform(0.60, 0.0, 180) };

		/**
		 * The {@code Pose2d}s of the robot relative to the {@code Pose2d} of the target
		 * {@code AprilTag} to align the robot to the right of that {@code AprilTag}.
		 */
		static Transform2d[] kRobotToTagsRightReady = { transform(1.1, 0.5, 180),
				transform(0.60, 0.0, 180) };

		/**
		 * A {@code Map} storing the distance to move forward to score at each scoring
		 * level.
		 */
		static Map<Integer, Double> kLevelForwardAdjustments = Map.of(
				1, 0.13, // L1
				2, 0.2, // L2
				3, 0.2, // L3
				4, 0.05); // L4
		// TODO: Check
		// increase the offset value to get closer to the tag

		/**
		 * A {@code Map} storing the elevator height for each scoring level.
		 */
		static Map<Integer, Double> kLevelElevatorHeights = Map.of(
				1, Units.inchesToMeters(3), // L1
				2, Units.inchesToMeters(8), // L2
				3, Units.inchesToMeters(29), // L3
				4, Units.inchesToMeters(48 + 2)); // L4
		// TODO: Check

		/**
		 * A {@code Map} storing the wrist angle for each scoring level.
		 */
		static Map<Integer, Double> kLevelWristAngles = Map.of(
				1, 221.0, // L1
				2, 221.0, // L2
				3, 240.0, // L3
				4, 223.0); // L4
		// TODO: Check

		/**
		 * A {@code Map} storing the additional distance to move forward/backward for
		 * some {@code AprilTag}s (positive: closer to the tag).
		 */
		static Map<Integer, Double> kTagForwardAdjustments = Map.of();
		// Map.of(18, 0.015, 22, 0.01);

		/**
		 * A {@code Map} storing the additional distance to move to left/right for
		 * some {@code AprilTag}s (positive: strafe left when facing toward the tag).
		 */
		static Map<Integer, Double> kTagSideAdjustments = Map.of();
		// Map.of(19, -0.02, 21, -0.01, 22, -0.01);

	}

}
