// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.util.datalog.StringLogEntry;
//import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	/** Creates a new SwerveDrive. */
	private swervelib.SwerveDrive drivetrain;
	private double driverScalingFactor = OperatorConstants.DEFUALT_DRIVER_SCALING_FACTOR;
	// private final StringLogEntry logger;

	public Drivetrain() {

		AutoBuilder.configureHolonomic(
				this::getPose2d, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
						new PIDConstants(Constants.AutoConstants.LINEAR_AUTO_KP, Constants.AutoConstants.LINEAR_AUTO_KI, Constants.AutoConstants.LINEAR_AUTO_KD), // Translation PID constants
						new PIDConstants(Constants.AutoConstants.ROTATIONAL_AUTO_KP, Constants.AutoConstants.ROTATION_AUTO_KI, Constants.AutoConstants.ROTATION_AUTO_KD), // Rotation PID constants
						4.5, // Max module speed, in m/s
						0.4, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
		// logger = new StringLogEntry(DataLogManager.getLog(), "/swerve-drive");
		try {
			SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
			drivetrain = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
					.createSwerveDrive(SwerveConstants.MAX_SPEED);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		drivetrain.setHeadingCorrection(false);

	}
	// TODO: All variable declarations should be before the constructor

	// public SwerveAutoBuilder autoBuilder = null;
	// ^^^^^^^ this was a duplicate that I didnt want to get rid of in case it was
	// correct
	@Override
	public void periodic() {
		// Odometry updates is not run in periodic anymore, it automaticly runs in a
		// Notifier by YAGSL

		SmartDashboard.putNumber("X coordinate", getPose().getX());
		SmartDashboard.putNumber("Y coordinate", getPose().getY());
	}

	/**
	 * The primary method for controlling the drivebase. Takes a
	 * {@link Translation2d} and a rotation rate, and
	 * calculates and commands module states accordingly. Can use either open-loop
	 * or closed-loop velocity control for
	 * the wheel velocities. Also has field- and robot-relative modes, which affect
	 * how the translation vector is used.
	 *
	 * @param translation   {@link Translation2d} that is the commanded linear
	 *                      velocity of the robot, in meters per
	 *                      second. In robot-relative mode, positive x is torwards
	 *                      the bow (front) and positive y is
	 *                      torwards port (left). In field-relative mode, positive x
	 *                      is away from the alliance wall
	 *                      (field North) and positive y is torwards the left wall
	 *                      when looking through the driver station
	 *                      glass (field West).
	 * @param rotation      Robot angular rate, in radians per second. CCW positive.
	 *                      Unaffected by field/robot
	 *                      relativity.
	 * @param fieldRelative Drive mode. True for field-relative, false for
	 *                      robot-relative.
	 * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true
	 *                      to disable closed-loop.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		drivetrain.drive(translation, rotation, fieldRelative, isOpenLoop);
	}

	/**
	 * The primary method for controlling the drivebase. Takes a
	 * {@link Translation2d} and a rotation rate, and
	 * calculates and commands module states accordingly. Can use either open-loop
	 * or closed-loop velocity control for
	 * the wheel velocities. Also has field- and robot-relative modes, which affect
	 * how the translation vector is used.
	 *
	 * @param translation       {@link Translation2d} that is the commanded linear
	 *                          velocity of the robot, in meters per
	 *                          second. In robot-relative mode, positive x is
	 *                          torwards the bow (front) and positive y is
	 *                          torwards port (left). In field-relative mode,
	 *                          positive x is away from the alliance wall
	 *                          (field North) and positive y is torwards the left
	 *                          wall when looking through the driver station
	 *                          glass (field West).
	 * @param rotation          Robot angular rate, in radians per second. CCW
	 *                          positive. Unaffected by field/robot
	 *                          relativity.
	 * @param fieldRelative     Drive mode. True for field-relative, false for
	 *                          robot-relative.
	 * @param isOpenLoop        Whether to use closed-loop velocity control. Set to
	 *                          true to disable closed-loop.
	 * @param headingCorrection Whether to use code to maintain current heading.
	 *                          True to enable heading correction.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
			boolean headingCorrection) {
		drivetrain.setHeadingCorrection(headingCorrection);
		drivetrain.drive(translation, rotation, fieldRelative, isOpenLoop);
		drivetrain.setHeadingCorrection(false);
		// if(translation.toString() == null){
		// logger.append("translation is null");
		// }
		// else{
		// logger.append("Translation: " + translation.toString() + ". Rotation: " +
		// rotation + ". Field Relative: " + fieldRelative + ". Heading Correction: " +
		// headingCorrection);
		// }

	}

	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveKinematics2} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics() {
		return drivetrain.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not
	 * need to be reset when calling this
	 * method. However, if either gyro angle or module position is reset, this must
	 * be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		drivetrain.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by
	 * odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose() {
		return drivetrain.getPose();
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		drivetrain.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		drivetrain.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but
	 * facing toward 0.
	 */
	public void zeroGyro() {
		drivetrain.zeroGyro();
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		drivetrain.setMotorIdleMode(brake);
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the imu. CCW
	 * positive, not wrapped.
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading() {
		return drivetrain.getYaw();
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for
	 * speeds in which direction. The other for
	 * the angle of the robot.
	 *
	 * @param xInput   X joystick input for the robot to move in the X direction.
	 * @param yInput   Y joystick input for the robot to move in the Y direction.
	 * @param headingX X joystick which controls the angle of the robot.
	 * @param headingY Y joystick which controls the angle of the robot.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
		xInput = processJoystickInput(xInput);
		yInput = processJoystickInput(yInput);
		return drivetrain.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
				getHeading().getRadians(), SwerveConstants.MAX_SPEED);
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for
	 * speeds in which direction. The other for
	 * the rotational speed.
	 *
	 * @param xInput          X joystick input for the robot to move in the X
	 *                        direction.
	 * @param yInput          Y joystick input for the robot to move in the Y
	 *                        direction.
	 * @param deltaThetaInput X joystick which controls the rate of turning of the
	 *                        robot.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	// based on the current heading and what the desired heading is?
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double deltaThetaInput) {
		xInput = processJoystickInput(xInput) * SwerveConstants.MAX_SPEED;
		yInput = processJoystickInput(yInput) * SwerveConstants.MAX_SPEED;
		deltaThetaInput = processJoystickInput(deltaThetaInput) * drivetrain.swerveController.config.maxAngularVelocity;

		return drivetrain.swerveController.getRawTargetSpeeds(xInput, yInput, deltaThetaInput);
	}

	private double processJoystickInput(double input) {
		return Math.pow(input, 3) * driverScalingFactor;
	}

	public void setDriverScalingFactor(double factor) {
		driverScalingFactor = factor;
	}

	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot
	 *
	 * @return A ChassisSpeeds object of the current field-relative velocity
	 */
	public ChassisSpeeds getFieldVelocity() {
		return drivetrain.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity() {
		return drivetrain.getRobotVelocity();
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController() {
		return drivetrain.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} fpr the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return drivetrain.swerveDriveConfiguration;
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock() {
		drivetrain.lockPose();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch() {
		return drivetrain.getPitch();
	}

}
