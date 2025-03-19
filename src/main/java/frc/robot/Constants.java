package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
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

		public static final double kRotationP = 5; // TODO: tune it
		public static final double kRotationI = 0.0;
		public static final double kRotationD = 0.1; // TODO: tune it
		public static final double kRotationS = 0;
		public static final double kRotationV = 1.9;
		public static final double kRotationA = 0.009;

		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kDriveGearRatio = 6.75;
		public static final double kSteerGearRatio = 150.0 / 7;
		public static final double kWheelDiameter = Units.inchesToMeters(4);
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

		public static final double kTeleopDriveMaxSpeed = 12.0; // 5 meters per second
		public static final double kTeleopTurnMaxAngularSpeed = Math.toRadians(360 * 5);

		public static final double kDriveMaxSpeed = 12.0; // 5 meters per second
		public static final double kDriveMinSpeed = 0.2; // 0.2 meters per second
		public static final double kTurnMaxAngularSpeed = Math.toRadians(360); // 1 rotation per second
		public static final double kTurnMinAngularSpeed = Math.toRadians(0); // 0 degree per second

		// DriveCommand.java Constants
		public static final double kDriveP = 5;
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxAcceleration = 2 * kDriveMaxSpeed; // kDriveMaxSpeed in 1.5 sec

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

		public static final double kMaxExtension = Units.inchesToMeters(49.5 + 0.75);
		public static final double kLevelOneHeight = Units.inchesToMeters(3);
		public static final double kLevelTwoHeight = Units.inchesToMeters(8);
		public static final double kLevelThreeHeight = Units.inchesToMeters(29);
		public static final double kLevelFourHeight = Units.inchesToMeters(48 + 2); // TODO: 72 from carpet
		public static final double kCoralStationHeight = Units.inchesToMeters(17 + 2);
		public static final double kLevelOneClearanceHeight = Units.inchesToMeters(0.7);
		// now implemented here
		public static final double kLevelTwoClearanceHeight = Units.inchesToMeters(0.7);

		public static final double kAlgaeLevelThreeHeight = Units.inchesToMeters(0.25);
		public static final double kAlgaeLevelTwoHeight = Units.inchesToMeters(14);
	}

	public static final class WristConstants {
		public static final int kWristMotorPort = 27;
		public static final int kSmartCurrentLimit = 20;
		public static final int kSecondaryCurrentLimit = 20;

		public static final int kGrabberAngleLevelFour = 223; // 228 with wrist offset
		public static final int kGrabberAngleLevelThree = 240; // 232 with wrist offset
		public static final int kGrabberAngleLevelOneAndTwo = 221;
		public static final int kAlgaeAngle = 170;
		public static final int kBaseAngle = 272; // TODO: 270?

		public static final double kWristForwardSoftLimit = 274; // Wrist facing down //TODO: Redo 0
		public static final double kWristReverseSoftLimit = 90; // Wrist facing up
		public static final double kWristOffset = 0.75; // 3.5/260 for offset

		public static final double kP = 0.015;
		public static final double kI = 0.0;
		public static final double kD = 0;

		public static final double kTolerance = 4;
	}

	public static final class AutoConstants {
		public static AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

		// TODO tune to our bot
		// from:
		// https://github.com/FRCTeam3044/2025swervebase/blob/63305be2c48e7f89c1c2cb156e06987dd0aecc72/src/main/java/frc/robot/subsystems/vision/VisionConstants.java
		public static double maxAmbiguity = 0.2;
		public static double maxZError = 0.75;

		// Standard deviation baselines, for 1 meter distance and 1 tag
		// (Adjusted automatically based on distance and # of tags)
		public static double linearStdDevBaseline = 0.1; // Meters | 0.02
		public static double angularStdDevBaseline = 0.06; // Radians

		// Standard deviation multipliers for each camera
		// (Adjust to trust some cameras more than others)
		public static double[] cameraStdDevFactors = new double[] {
				0.8, // Camera 0 | 1
				1.6, // Camera 1 | 1
		};

		// Multipliers to apply for MegaTag 2 observations
		public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
		public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

		/**
		 * The {@code Transform3d} expressing the pose of the first camera relative to
		 * the pose of the robot.
		 */
		public static Transform3d kRobotToCamera1 = new Transform3d(new Translation3d(0.3, 0.0, 0.2),
				new Rotation3d(0, Units.degreesToRadians(-10), 0));

		/**
		 * The {@code Transform3d} expressing the pose of the second camera relative to
		 * the pose of the robot.
		 */
		public static Transform3d kRobotToCamera2 = new Transform3d(new Translation3d(-0.5, -0.0, 0.2),
				new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));

		public static Transform2d kCameraToTwentyOne = new Transform2d(0.75, 0.1643126,
				Rotation2d.fromDegrees(180));
	}
}