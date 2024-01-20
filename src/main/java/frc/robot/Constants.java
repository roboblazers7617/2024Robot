// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class TestNumber {
		public static int number = 5;
	}

	public static class ClimberConstants {
		public static final int LEFT_MOTOR_ID = 500;// TODO
		public static final int RIGHT_MOTOR_ID = 499;// TODO

		public static double MAX_HEIGHT = 100;
		public static final double KS = 0;
		public static final double KG = 0;
		public static final double KV = 0;
		public static final double KP = 0;
		public static final double KI = 0;
		public static final double KD = 0;
		public static final double POSITION_CONVERSION_FACTOR = 0;
		public static final double VELOCITY_CONVERSION_FACTOR = 0;
		public static final double ABS_POSITION_CONVERSION_FACTOR = 0;
		public static final double ABS_VELOCITY_CONVERSION_FACTOR = 0;
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class AutoConstants {

	}

	public static class SwerveConstants {

	}

	public static class IntakeConstants {

		public static final int MOTOR_CAN_ID = 20;
		public static final int SENSOR_DIO = 1;
		public static final double INTAKE_SPEED = 1;
		public static final double OUTAKE_SPEED = -1;
	}

	public static class PivotConstants {
		public enum PivotPosition {
			STOWED(0);

			PivotPosition(double angle) {
				this.angle = angle;
			}

			private final double angle;

			public double angle() {
				return angle;
			}
		}
	}

	public static class ShooterConstants {
		public enum ShootingPosition {
			SUBWOOFER(0);

			ShootingPosition(int rpm) {
				this.rpm = rpm;
			}

			private final int rpm;

			public int rpm() {
				return rpm;
			}
		}

		public static final int IDLE_SPEED = 0;

	}
}
