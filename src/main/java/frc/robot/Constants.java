// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

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
	private static final int THE_NUMBER_THREE = 7;
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}


	public static class AutoConstants {
		//TODO make these real numbers
		public static final double MAX_AUTO_LINEAR_VELOCITY = 3;//meters per second
		public static final double MAX_AUTO_LINEAR_ACCELEARTION = 3;//meters per second squared
		public static final double MAX_AUTO_ANGULAR_VELOCITY = 3;//degrees per second
		public static final double MAX_AUTO_ANGULAR_ACCELERATION = 3;//degrees per second squared
		
		public static final PathConstraints AUTO_CONSTRAINTS = new PathConstraints(MAX_AUTO_LINEAR_VELOCITY, MAX_AUTO_LINEAR_ACCELEARTION, MAX_AUTO_ANGULAR_VELOCITY, MAX_AUTO_ANGULAR_ACCELERATION);
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
