package frc.robot;

public class Constants {
	public static final class ElevatorConstants {

		public static final double kFF = 0.00008040000102482736; // Feedforward Constant
		public static final int kElevatorMotorLeftPort = 45;
		public static final int kElevatorMotorRightPort = 44;
		public static final boolean kLeftInvert = false;
		public static final boolean kRightInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kMinOutput = -1;
		public static final double kMaxOutput = 1;
		public static final double kP = 5;
		public static final double kI = 100000;
		public static final double kD = 0;
		public static final double kTolerance = 1;
		public static final int kMaxExtension = 250;
	}
}
