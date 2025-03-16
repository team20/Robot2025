// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {
	/** The USB port that's used to connect to the Arduino. */
	private SerialPort m_usb;

	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 0),
		RAINBOW_PARTY_FUN_TIME((byte) 1),
		INTAKED_ALGAE((byte) 2),
		ATTACHED_CLIMBER((byte) 3),
		BLINKING_RED((byte) 6),
		DEFAULT((byte) 20);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}

	/** Creates a new ArduinoSubsystem. */
	public ArduinoSubsystem() {
		try {
			m_usb = new SerialPort(9600, Port.kMXP);
		} catch (Exception e) {
			DriverStation.reportError("Could not initialize Arduino over USB", false);
			m_usb = null;
		}
		setCode(StatusCode.DEFAULT);
	}

	public void setCode(StatusCode code) {
		if (m_usb != null) {
			m_usb.write(new byte[] { code.code }, 1);
		}
	}

	/**
	 * Changes the led to the code
	 * 
	 * @param code the status code of the LEDs
	 * @return runs the command once
	 */
	public Command ledPattern(StatusCode code) {
		return runOnce(() -> setCode(code));
	}

	/**
	 * Changes the leds to the code for a certain time
	 * 
	 * @param code the status coed of the LEDs
	 * @param seconds the time for the command to run
	 * @return runs the command until timeout
	 */
	public Command ledPatternTime(StatusCode code, double seconds) {
		return run(() -> setCode(code)).withTimeout(seconds);
	}
}