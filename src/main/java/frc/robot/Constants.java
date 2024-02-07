// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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

	public static class ArmConstants {
		public static final int LEFT_MOTOR_ID = 6; // for test arm
		public static final int RIGHT_MOTOR_ID = 499;// TODO

		public static double MAX_HEIGHT = 0;
		public static final double KS = 0;
		public static final double KG = 0.1;
		public static final double KV = 0;
		public static final double KP = 0.004;
		public static final double KI = 0;
		public static final double KD = 0;
		public static final double kIz = 0; //izone
		public static final double kMinOutput = -1;
		public static final double kMaxOutput = 1;
		public static final double minVel = 0;
		// public static final double POSITION_CONVERSION_FACTOR = 16;
		// public static final double VELOCITY_CONVERSION_FACTOR = 0;
		public static final double GEAR_RATIO = 1.0/16.0;
		public static final double ABS_POSITION_CONVERSION_FACTOR = 360;
		public static final double ABS_VELOCITY_CONVERSION_FACTOR = ABS_POSITION_CONVERSION_FACTOR / 60; // what is this

		public static final int SLOT_ID = 0;
		public static final double MAX_VELOCITY = 15;//was 1
		public static final double MAX_ACCELERATION = 15;
		public static final double MAX_ERROR = 0.02;
	}

	public static class OperatorConstants {
		public static final double JOYSTICK_DEADBAND = 0.1;
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final double DEFUALT_DRIVER_SCALING_FACTOR = 0.5;
	}

	public static class AutoConstants {

	}

	public static class SwerveConstants {

		/**
		 * This is the time it takes for the PID to update in seconds, 20ms + 110ms sprk
		 * max velocity lag
		 */
		public static final double LOOP_TIME = 0.13;
		/**
		 * The total mass of the robot, in kilograms. Includes Batteries and bumper
		 * weight
		 */
		public static final double ROBOT_MASS = Units.lbsToKilograms(85.3);

		/**
		 * The total mass of the drivetrain, eboard, and associated components. Position
		 * reflects the aproximate center of mass, relative to the floor, in meters
		 */
		public static final Matter DRIVEBASE = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3.5)),
				ROBOT_MASS);
		/** Time to brake the chassis for after the robot is disabled, in seconds */
		public static final double BRAKE_TIMER_DURATION = 10;
		public static final double MAX_SPEED =0.4;
		public static final double ROTATION_MULTIPLIER = 0.4; 

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

	public static final double BATTERY_WARNING_VOLTAGE = 11;
	public static final int NUMBER_OF_MOTORS = 10;
}
