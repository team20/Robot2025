package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final class AlgaeConstants {
		// FLYWHEEL IS 550
		// OTHER IS NORMAL NEO

		public static final int kFlywheelMotorPort = 90;
		public static final int kGrabberAnglePort = 3;
		public static final boolean kFlywheelInvert = false;
		public static final boolean kGrabberAngleInvert = false;

		public static final int kSmartCurrentLimit = 50;
		public static final int kPeakCurrentDurationMillis = 100;

		public static final int k550SmartCurrentLimit = 15;

		public static final double kCurrentToStop = 20;
		public static final double kTimeOverCurrentToStop = .01;

		public static final double kP = 0.5;
		public static final double kI = 0.0;
		public static final double kD = 0;
	}

	public static final class CheeseStickConstants {
		public static final int kServoPort = 0;
		public static final int kMaxRotation = 270;
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

		// Make sure these are tuned (can do with SysId)
		public static final double kP = 0.09;
		public static final double kI = 0.0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kV = 0.11;
		public static final double kA = 0.009;

		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kDriveGearRatio = 6.12;
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
		public static final double kFF = 0.00008040000102482736; // Feedforward Constant
		public static final int kElevatorMotorPort = 45;
		public static final int kSmartCurrentLimit = 60;
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kMinOutput = -1;
		public static final double kMaxOutput = 1;
		public static final double kP = 5;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kTolerance = 1;
		public static final int kMaxExtension = 250;
		public static final int kLevelOneHeight = 0; /* same as level 2 */
		public static final int kLevelTwoHeight = 0;
		public static final double kLevelThreeHeight = 70 / 2.54;
		public static final int kLevelFourHeight = 0;
		public static final int kCoralStationHeight = 0;
	}

	public static final class WristConstants {
		public static final int kWristMotorPort = 1; /* 60 */
		public static final int kSmartCurrentLimit = 20;
		public static final int kSecondaryCurrentLimit = 20;

		// Make sure these are tuned (can do with SysId)
		public static final double kP = 0.09;
		public static final double kI = 0.0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kG = 0;
		public static final double kV = 0.11;
		public static final double kA = 0.009;

		public static final double kTolerance = 0;
	}
}
