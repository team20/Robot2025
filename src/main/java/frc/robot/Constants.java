package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {
	public static final class AlgaeConstants {
		public static final int kFlywheelMotorPort = 55;
		public static final int kGrabberAnglePort = 56;
		public static final boolean kFlywheelInvert = true;
		public static final boolean kGrabberAngleInvert = false;

		public static final int k550SmartCurrentLimit = 15;

		public static final int kSmartCurrentLimit = 50;
		public static final int kPeakCurrentDurationMillis = 100;

		public static final double kCurrentToStop = 20;
		public static final double kTimeOverCurrentToStop = .01;

		public static final float kDeployGrabberRotations = 2;
		public static final double kFlywheelSpeed = .8;

		public static final double kP = 0.5; // TODO: Tune
		public static final double kI = 0.0;
		public static final double kD = 0;
	}

	public static final class CheeseStickConstants {
		public static final int kServoPort = 0; // TODO: CHANGE
		/**
		 * Set this value to how far the cheese stick wheels extend beyond the lexan.
		 */
		public static final Distance kExtensionLength = Inch.of(.5);
	}

	public static final class ClimberConstants {
		public static final int kClimberMotorPort = -1; // TODO: Add actual motor ID
		public static final double kSpeed = 0.5;
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
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kV = 0.11;
		public static final double kA = 0.009;

		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kDriveGearRatio = 6.75;
		public static final double kSteerGearRatio = 150.0 / 7;
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio;

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		// TODO: CHECK
		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

		public static final int kEncoderDepth = 4;
		public static final int kEncoderMeasurementPeriod = 16;
		public static final int kSteerSmartCurrentLimit = 60;
		public static final int kSteerPeakCurrentLimit = kSteerSmartCurrentLimit + 15;
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
	}

	public static final class ElevatorConstants {
		public static final int kElevatorMotorPort = 57; // TODO: Change with real one
		public static final int kSmartCurrentLimit = 60;
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kP = 0; // TODO: tune
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kG = 2;
		public static final double kV = 2;
		public static final double kA = 0;
		public static final double kGearRatio = 10; // TODO: Right gear ratio?
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
		public static final double kMetersPerMotorRotation = (1 / kGearRatio)
				* (2 * Math.PI * kMetersPerPulleyRotation);
		public static final double kMaxVelocity = 2;
		public static final double kMaxAccel = 2;
		public static final double kTolerance = 1;
		// public static final int kMaxExtension = 250; // TODO: Check this? Ask Design
		public static final double kLevelOneHeight = Units.inchesToMeters(41 - 24); // TODO: During testing make sure
																					// these are right
		public static final double kLevelTwoHeight = kLevelOneHeight; // same as level 1
		public static final double kLevelThreeHeight = Units.inchesToMeters(60 - 24);
		public static final double kLevelFourHeight = Units.inchesToMeters(90 - 24);
		public static final double kToScoreHeightDecrease = 0; // TODO: The amount that the elevator decreases in order
																// to
																// score
		public static final double kCoralStationHeight = 0; // TODO: Change
	}

	public static final class WristConstants {
		public static final int kWristMotorPort = 58; // TODO: Change
		public static final int kSmartCurrentLimit = 20;
		public static final int kSecondaryCurrentLimit = 20;

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.09;
		public static final double kI = 0.0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kG = 0;
		public static final double kV = 0.11;
		public static final double kA = 0.009;

		public static final double kTolerance = 0; // TODO: Change this
	}
}
