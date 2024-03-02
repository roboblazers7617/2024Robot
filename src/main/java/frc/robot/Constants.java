// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public final static double ENABLED_BATTERY_WARNING_VOLTAGE = 9.9;
	public final static double DISABLED_BATTERY_WARNING_VOLTAGE = 12.2;

	public static class TestNumber{
		public static int number = 5;
	}

	public static class ArmConstants {
		public static final int RIGHT_MOTOR_ID = 25; 
		public static final int LEFT_MOTOR_ID = 26;
		public static final int MAX_AMPERAGE = 40;

		// Some relavant numbers
		// arm mass: 35-40 lbs
		// center of mass distance
		// when retracted: 14.04112
		// when extended: 17.64693

		// stow values
		// arm: 20.4
		// elevator: retracted



		public static final double MAX_ANGLE = 90;
		public static final double MIN_ANGLE = 10;
		/** the mininum angle the arm can be where the elevator can pass over the bumper */
		public static final double MIN_ABOVE_PASS_ANGLE = 25;

		// Constants for extended state
		public static final double EXTENDED_KS = 0; //these values are based on the test arm
		public static final double EXTENDED_KG = 0.5; // .5 while extended
		public static final double EXTENDED_KV = 0.0;
	
		// Constants for retracted state
		public static final double RETRACTED_KS = 0; //these values are based on calculator
		public static final double RETRACTED_KG = 0.4;
		public static final double RETRACTED_KV = 0.0;
		public static final double KP = 0.025;
		public static final double KI = 0;
		public static final double KD = 0;
		public static final double kMinOutput = -0.25;
		public static final double kMaxOutput = 1;
		public static final double minVel = 0;
		// public static final double POSITION_CONVERSION_FACTOR = 16;
		// public static final double VELOCITY_CONVERSION_FACTOR = 0;
		public static final double GEAR_RATIO = 1/100.0;
		public static final double ABS_POSITION_CONVERSION_FACTOR = 360;
		public static final double ABS_VELOCITY_CONVERSION_FACTOR = ABS_POSITION_CONVERSION_FACTOR / 60;
		public static final double MAX_MANNUAL_ARM_SPEED = 45.0; 


		
		
	}

	public static class ElevatorConstants {
		public static final int RIGHT_MOTOR_ID = 28;
		public static final int LEFT_MOTOR_ID = 27;
		public static final int MAX_AMPERAGE = 40;
		public static final double KS = 0;
		public static final double KG = 0.0;
		public static final double KV = 0.0;
		public static final double KP = 1.5; // was .2
		public static final double KI = 0;
		public static final double KD = 0;
		public static final double kIz = 0; //izone
		public static final double kMinOutput = -1;
		public static final double kMaxOutput = 1;
		public static final double minVel = 0;
		public static final double GEAR_RATIO = 1/12;
		public static final double ABS_POSITION_CONVERSION_FACTOR = 360;
		public static final double ABS_VELOCITY_CONVERSION_FACTOR = ABS_POSITION_CONVERSION_FACTOR / 60;

		
		/** the maximum height the elevator can safely reach */
		public static final double MAX_HEIGHT = 18.6;
		/** the mininum height the elevator can be to safely reach over the bumper of the robot */
		public static final double MIN_ABOVE_PASS_HEIGHT = 18.6;
		/** the mininum height the elevator reaches when its retracted */
		public static final double MIN_HEIGHT = 0;
		/** the maximum height the elevator can be, but still be safely inside the bumber of the robot */
		public static final double MAX_BELOW_PASS_HEIGHT = 10; // REMOVE THIS, IT DOES NOT EXIST
		public static final int RIGHT_POTIENTIOMETER_PORT = 3;
		// public static final int LEFT_POTIENTIOMETER_PORT = 45;
		public static final double MAX_MANUAL_SPEED = 20;
		/** for potentiometer */
		public static final double OFFSET = 0.999;

	}

	public static class OperatorConstants {
		public static final double JOYSTICK_DEADBAND = 0.075;
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		public static final double ROTATION_RATE = 2;
	}
	
	public static class AutoConstants {
		public static final double LINEAR_AUTO_KP = 5.7524;
		public static final double LINEAR_AUTO_KI = 0;
		public static final double LINEAR_AUTO_KD = 0;
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
		public static final Matter DRIVEBASE = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3.5)), ROBOT_MASS);
		/** Time to brake the chassis for after the robot is disabled, in seconds */
		public static final double BRAKE_TIMER_DURATION = 10;
		public static final double MAX_VELOCITY_METER_PER_SEC = Units.feetToMeters(14.5);
		public static final double SLOW_SPEED_DECREMENT = 0.3;
		public static final double REGULAR_SPEED = 0.6;
		public static final double FAST_SPEED_INCREMENT = .2;
		public static final double PRECISE_INCREMENT = .1;
	}
	
	public static class IntakeConstants {
		public static final int MOTOR_BOTTOM_CAN_ID = 21;
		public static final int MOTOR_TOP_CAN_ID = 20;
		
		public static final int NOTE_POSSESSION_SENSOR_DIO = 1;
		public static final int NOTE_ALIGNMENT_SENSOR_DIO = 2;
		
		public static final double INTAKE_SPEED = 0.25;
		public static final double OUTAKE_SPEED = -0.25;
		public static final double ALIGNMENT_SPEED = 0.1; // What speed should the intake spin to align the note?
		public static final double FEEDER_SPEED = 0.25; // What speed should a note be fed into the shooter at?
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
		public static final int MOTOR_BOTTOM_CAN_ID = 23;
		public static final int MOTOR_TOP_CAN_ID = 22;
		
		public static final TunableNumber kP = new TunableNumber("Head/Shooter", "kP", 0.00005);
		public static final TunableNumber kI = new TunableNumber("Head/Shooter", "kI", 0);
		public static final TunableNumber kD = new TunableNumber("Head/Shooter", "kD", 0);
		public static final TunableNumber kMinOutput = new TunableNumber("Head/Shooter", "kMinOutput", -1);
		public static final TunableNumber kMaxOutput = new TunableNumber("Head/Shooter", "kMaxOutput", 1);
		
		public static final double IDLE_SPEED = 0;

		public static final TunableNumber VELOCITY_MINIMUM = new TunableNumber("Head/Shooter", "Velocity Threshold Minimum", 0);
		public static final TunableNumber VELOCITY_MAXIMUM = new TunableNumber("Head/Shooter", "Velocity Threshold Maximum", 0);
	}
	
	public static final double BATTERY_WARNING_VOLTAGE = 11;
	public static final int NUMBER_OF_MOTORS = 10;
	
	public static class VisionConstants {
		public static final Transform3d INTAKE_CAMERA_POSITION = new Transform3d(Units.inchesToMeters(14.0 + 3.0 / 4.0), Units.inchesToMeters(0.0), Units.inchesToMeters(6.0 + 7.0 / 16.0), new Rotation3d(0, Units.degreesToRadians(-64.0), 0));
		public static final Transform3d SHOOTER_CAMERA_POSITION = new Transform3d(-Units.inchesToMeters(14.0 + 3.0 / 4.0), Units.inchesToMeters(0.0), Units.inchesToMeters(6.0 + 7.0 / 16.0), new Rotation3d(0, Units.degreesToRadians(58.0), Units.degreesToRadians(180.0)));
	}
}