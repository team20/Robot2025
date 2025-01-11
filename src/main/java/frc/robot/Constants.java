package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			/** Left middle button */
			public static final int kSquare = 1;
			/** Bottom button */
			public static final int kX = 2;
			/** Right middle button */
			public static final int kCircle = 3;
			/** Top button */
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kLeftTrigger = 7;
			public static final int kRightTrigger = 8;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {

		public static final double kP = 1; // TODO: Update ALL
		public static final double kI = 1;
		public static final double kD = 1;
		public static final double kIz = 1;

		// PORTS
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 20;
		public static final int kFrontLeftSteerPort = 21;
		public static final int kBackRightDrivePort = 30;
		public static final int kBackRightSteerPort = 31;
		public static final int kBackLeftDrivePort = 40;
		public static final int kBackLeftSteerPort = 41;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;

		public static final double kDriveP = 0.7;
		public static final double kDriveI = 0;
		public static final double kDriveD = 0;
		public static final double kDriveMaxVelocity = 7;
		public static final double kDriveMaxAcceleration = 7;

		public static final double kTurnP = 0;
		public static final double kTurnI = 0;
		public static final double kTurnD = 0;
		public static final double kTurnMaxVelocity = 90;
		public static final double kTurnMaxAcceleration = 90;

		public static final double kMaxSpeed = 12; // in Volts (max volts = 12)
		public static final double kMinSpeed = 1;
		public static final double kTeleopMaxVoltage = 12;
		public static final double kTeleopMaxTurnVoltage = 7.2;
		public static final double kGearRatio = 6.12;
		public static final double kWheelDiameter = Units.inchesToMeters(4);

		public static final double kMetersPerMotorRotations = (1 / kGearRatio) * (Math.PI * kWheelDiameter);
		// TODO: Update name?

		public static final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381); // TODO:UPDATE COORDS
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
		public static final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

		public static final int kDriveSmartCurrentLimit = 45;
		public static final int kDrivePeakCurrentLimit = kDriveSmartCurrentLimit + 10;
		public static final int kSteerSmartCurrentLimit = 60;
		public static final int kSteerPeakCurrentLimit = kSteerSmartCurrentLimit + 15;
		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1; // .1
	}
}