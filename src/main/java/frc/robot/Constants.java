// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
	private final static double THE_NUMBER_THREE = 7;
	public final static double DISABLED_BATTERY_WARNING_VOLTAGE = 12.2;
	/** in meters */
	public final static double PODIUM_DISTANCE = 5.0;
	public static final int BRAKE_TOGGLE_BUTTON_DIO = 7;
	
	// public static class TestNumber{
	// public static int number = 5;
	// }
	
	public static class ShootingConstants {
		public enum ShootingPosition {
			// TODO: Rename DBOT to MID_STAGE to be more descriptive
			AMP(1500.0, 84.5, ElevatorConstants.MAX_HEIGHT), SUBWOOFER(5500.0, 14.0, ElevatorConstants.MAX_HEIGHT), PODIUM(7000.0, 32.0, ElevatorConstants.MIN_HEIGHT), // todo where should elevator be?
			DBOT(8000.0, 37.520, ElevatorConstants.MIN_HEIGHT); // todo where should elevator be?
			
			ShootingPosition(double rpm, double arm_angle, double elevator_target) {
				this.rpm = rpm;
				this.arm_angle = arm_angle;
				this.elevator_target = elevator_target;
			}
			
			private final double rpm;
			private final double arm_angle;
			private final double elevator_target;
			
			public double rpm() {
				return rpm;
			}
			
			public double arm_angle() {
				return arm_angle;
			}
			
			public double elevator_target() {
				return elevator_target;
			}
		}
		
		public static final int AUTO_SHOOT_SPEED = 6000;
		public static final int VARIABLE_DISTANCE_SHOT = 6750;
	}
	
	public static class ArmConstants {
		public static final int RIGHT_MOTOR_ID = 25;
		public static final int LEFT_MOTOR_ID = 26;
		public static final int MAX_AMPERAGE = 40;
		public static final double ARM_OFFSET = 236.04;
		
		// Some relavant numbers
		// arm mass: 35-40 lbs
		// center of mass distance
		// when retracted: 14.04112
		// when extended: 17.64693
		
		// stow values
		// arm: 20.4
		// elevator: retracted
		
		// all arm angle targets must be ~ 4 degrees more then what it should actually be
		public static final double MAX_ANGLE = 90;
		public static final double MIN_ANGLE = 2.5;
		public static final double SOURCE_ANGLE = 64.5; // 64 to compensate for change of ABEncoder offset hack;
		public static final double FLOOR_PICKUP = 3.75;
		public static final double STOW_ANGLE = 20.0;
		/** the mininum angle the arm can be where the elevator can pass over the bumper */
		public static final double MIN_ABOVE_PASS_ANGLE = 18.5;
		
		// Constants for extended state
		public static final double EXTENDED_KS = 0.17; // these values are based on the test arm
		public static final double EXTENDED_KG = 0.28; // .5 while extended
		public static final double EXTENDED_KV = 0.2;
		
		// Constants for retracted state
		public static final double RETRACTED_KS = 0.17; // these values are based on calculator
		public static final double RETRACTED_KG = 0.3;
		public static final double RETRACTED_KV = 0.1;
		public static final double KP = 0.028; // was 0.015
		public static final double KI = 0;
		public static final double KD = 0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1;
		public static final double ABS_POSITION_CONVERSION_FACTOR = 360;
		public static final double ABS_VELOCITY_CONVERSION_FACTOR = ABS_POSITION_CONVERSION_FACTOR / 60;
		public static final double MAX_MANNUAL_ARM_SPEED = 45.0;
		public static final double ARM_VELOCITY_DEADBAND = 0.75;
		public static final double ARM_AT_TARGET_DEADBAND = 1.5;
	}
	
	public static class ElevatorConstants {
		public static final boolean KILL_IT_ALL = false;
		public static final int RIGHT_MOTOR_ID = 28;
		public static final int LEFT_MOTOR_ID = 27;
		public static final int MAX_AMPERAGE = 40;
		public static final double KS = 0.0; // 0.2
		public static final double KG = 0.0; // 0.1
		public static final double KV = 0.0;
		public static final double KP = 0.3; // was .2 more recently 1.9
		public static final double KI = 0;
		public static final double KD = 0.0;
		public static final double kMinOutput = -.35;
		public static final double kMaxOutput = 0.60;
		
		/** the maximum height the elevator can safely reach */
		public static final double MAX_HEIGHT = 18.6;
		/** the mininum height the elevator can be to safely reach over the bumper of the robot */
		public static final double MIN_ABOVE_PASS_HEIGHT = 18;
		/** the mininum height the elevator reaches when its retracted */
		public static final double MIN_HEIGHT = 1;
		/** the maximum height the elevator can be, but still be safely inside the bumber of the robot */
		public static final double MAX_MANUAL_SPEED = 20;
		public static final double ELEVATOR_AT_TARGET_DEADBAND = 2.0;
	}
	
	public static class OperatorConstants {
		public static final double DRIVER_JOYSTICK_DEADBAND = 0.075;
		public static final double OPERATOR_JOYSTICK_DEADBAND = 0.125;
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		public static final double ROTATION_RATE = 2;
	}
	
	public static class AutoConstants {
		public static final double LINEAR_AUTO_KP = 5.7960566071;
		public static final double LINEAR_AUTO_KI = 0;
		public static final double LINEAR_AUTO_KD = 0;
		public static final double MAX_MODULE_SPEED = 15;
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
		public static final double ROBOT_MASS = Units.lbsToKilograms(150);
		
		/**
		 * The total mass of the drivetrain, eboard, and associated components. Position
		 * reflects the aproximate center of mass, relative to the floor, in meters
		 */
		public static final Matter DRIVEBASE = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3.5)), ROBOT_MASS);
		/** Time to brake the chassis for after the robot is disabled, in seconds */
		public static final double BRAKE_TIMER_DURATION = 10;
		public static final double MAX_VELOCITY_METER_PER_SEC = Units.feetToMeters(14.5);
		public static final double SLOW_SPEED_DECREMENT = 0.3;
		public static final double THE_NUMBER_THREE_TWO = THE_NUMBER_THREE;
		public static final double REGULAR_SPEED = 0.8;
		public static final double FAST_SPEED_INCREMENT = .2;
		public static final double PRECISE_INCREMENT = .1;
		public static final double TURN_TO_TAG_RANGE_FOR_END = 1.0;
		public static final double TURN_TO_ANGLE_RANGE_FOR_END = 2.0;
		public static final double FAST_TURN_TIME = 2.0;
	}
	
	public static class IntakeConstants {
		public static final int MOTOR_CAN_ID = 21;
		
		public static final int NOTE_SENSOR_DIO = 9;
		public static final int NOTE_ALIGNMENT_SENSOR_DIO = 6;
		
		public static final double INTAKE_SPEED = 0.75; // 0.95
		public static final double ALIGMNMENT_SPEED = 0.2; // 0.08
		public static final double OUTAKE_SPEED = -0.25;
		public static final double FEEDER_SPEED = 0.25; // What speed should a note be fed into the shooter at?
	}
	
	public static class ShooterConstants {
		public static final int MOTOR_BOTTOM_CAN_ID = 23;
		public static final int MOTOR_TOP_CAN_ID = 22;
		
		public static final double BOTTOM_kP = 0.00026;
		public static final double BOTTOM_kI = 0;
		public static final double BOTTOM_kD = 0.0;
		public static final double BOTTOM_kF = 0.004;
		public static final double BOTTOM_kMinOutput = -1;
		public static final double BOTTOM_kMaxOutput = 1;
		
		public static final double TOP_kP = 0.00026;
		public static final double TOP_kI = 0;
		public static final double TOP_kD = 0.0;
		public static final double TOP_kF = 0.004;
		public static final double TOP_kMinOutput = -1;
		public static final double TOP_kMaxOutput = 1;
		
		public static final double VELOCITY_MINIMUM = 0.5;
		public static final double VELOCITY_MAXIMUM = 2.0;
	}
	
	public static class ClimberConstants {
		public static final int LEFT_CLIMBER_PORT = 30;
		public static final int RIGHT_CLIMBER_PORT = 31;
		public static final double MAX_ENCODER_VALUE = 100;
		public static final double BALANCE_KP = 0;
		public static final double BALANCE_KI = 0;
		public static final double BALANCE_KD = 0;
		public static final double CLIMB_HEIGHT = 60;
		public static final double CLIMB_RATE = .6;
	}
	
	public static final int NUMBER_OF_MOTORS = 10;
	
	public static class VisionConstants {
		public static final double MAX_DETECTION_RANGE = 5.5;// TODO 3.2
		public static final Transform3d INTAKE_CAMERA_POSITION = new Transform3d(Units.inchesToMeters(10.0 + 5.0 / 8.0), -Units.inchesToMeters(-(11.0 + 3.0 / 6.0)), Units.inchesToMeters(9.0 + 1.0 / 4.0), new Rotation3d(0, Units.degreesToRadians(-45), 0));
		public static final Transform3d SHOOTER_CAMERA_POSITION = new Transform3d(-Units.inchesToMeters(-(10.0 + 5.0 / 8.0)), Units.inchesToMeters(11.0 + 3.0 / 6.0), Units.inchesToMeters(9.0 + 1.0 / 4.0), new Rotation3d(0, Units.degreesToRadians(45), Units.degreesToRadians(180)));
	}
}
