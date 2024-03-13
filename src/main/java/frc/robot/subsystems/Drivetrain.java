// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.shuffleboard.MotorTab;

import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {
	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive swerveDrive;
	private final Vision vision;

	private final MotorTab motorTab = new MotorTab(8, "swerveDrive");

	private Optional<EstimatedRobotPose> visionMeasurement;
	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory
	 *            Directory of swerve drive config files.
	 */
	public Drivetrain(Vision vision) {
		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
		// objects being created.
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
		try {
			swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
					.createSwerveDrive(SwerveConstants.MAX_VELOCITY_METER_PER_SEC);
			// Alternative method if you don't want to supply the conversion factor via JSON
			// files.
			// swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
			// angleConversionFactor, driveConversionFactor);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
													// via angle.

  
	setupPathPlanner();
	this.vision = vision;

	for (int i = 0; i < 4; i++){
		motorTab.addMotor(new CANSparkMax[] {(CANSparkMax) swerveDrive.getModules()[i].getDriveMotor().getMotor()});
		motorTab.addMotor(new CANSparkMax[] {(CANSparkMax) swerveDrive.getModules()[i].getAngleMotor().getMotor()});
	}
  }



	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
						new PIDConstants(Constants.AutoConstants.LINEAR_AUTO_KP, Constants.AutoConstants.LINEAR_AUTO_KI, Constants.AutoConstants.LINEAR_AUTO_KD),
						// Translation PID constants
						new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p, swerveDrive.swerveController.config.headingPIDF.i, swerveDrive.swerveController.config.headingPIDF.d),
						// Rotation PID constants
						10,
						// Max module speed, in m/s
						swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
						// Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig()
				// Default path replanning config. See the API for the options here
				), () -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
					var alliance = DriverStation.getAlliance();
					return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
				}, this // Reference to this subsystem to set requirements
		);
	}
	
	/**
	 * Get the path follower with events.
	 *
	 * @param pathName
	 *            PathPlanner path name.
	 * @param setOdomToStart
	 *            Set the odometry position to the start of the path.
	 * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
	 */
	public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
		// Load the path you want to follow using its name in the GUI
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
		
		if (setOdomToStart) {
			resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
		}
		
		// Create a path following command using AutoBuilder. This will also trigger
		// event markers.
		return AutoBuilder.followPath(path);
	}
	
	/**
	 * Command to drive the robot using translative values and heading as a setpoint.
	 *
	 * @param translationX
	 *            Translation in the X direction. Cubed for smoother controls.
	 * @param translationY
	 *            Translation in the Y direction. Cubed for smoother controls.
	 * @param headingX
	 *            Heading X to calculate angle of the joystick.
	 * @param headingY
	 *            Heading Y to calculate angle of the joystick.
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
		// swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
		return run(() -> {
			double xInput = translationX.getAsDouble();
			double yInput = translationY.getAsDouble();
			// Make the robot move
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumVelocity()));
		});
	}
	
	/**
	 * Command to drive the robot using translative values and heading as a setpoint.
	 *
	 * @param translationX
	 *            Translation in the X direction.
	 * @param translationY
	 *            Translation in the Y direction.
	 * @param rotation
	 *            Rotation as a value between [-1, 1] converted to radians.
	 * @return Drive command.
	 */
	public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
		// swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
		return run(() -> {
			// Make the robot move
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble() * Math.PI, swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumVelocity()));
		});
	}
	
	/**
	 * Command to drive the robot using translative values and heading as angular velocity.
	 *
	 * @param translationX
	 *            Translation in the X direction. Cubed for smoother controls.
	 * @param translationY
	 *            Translation in the Y direction. Cubed for smoother controls.
	 * @param angularRotationX
	 *            Angular velocity of the robot to set. Cubed for smoother controls.
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
		return run(() -> {
			swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(), true, false);
		}).finallyDo(() -> {
			swerveDrive.swerveController.lastAngleScalar = getHeading().getRadians();
		});
	}
	
	public Command turnToAngleCommand(Rotation2d angle) {
		return Commands.run(() -> {
			this.driveFieldOriented(getTargetSpeeds(0, 0, angle));
		}, this).raceWith(Commands.waitUntil(() -> Math.abs(getHeading().minus(angle).getDegrees()) <= 2)).finallyDo(() -> {
			swerveDrive.swerveController.lastAngleScalar = getHeading().getRadians();
		});
	}
	
	/**
	 * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and
	 * calculates and commands module states accordingly. Can use either open-loop or closed-loop velocity control for
	 * the wheel velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation
	 *            {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
	 *            second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
	 *            torwards port (left). In field-relative mode, positive x is away from the alliance wall
	 *            (field North) and positive y is torwards the left wall when looking through the driver station
	 *            glass (field West).
	 * @param rotation
	 *            Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
	 *            relativity.
	 * @param fieldRelative
	 *            Drive mode. True for field-relative, false for robot-relative.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
	}
	
	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity
	 *            Velocity according to the field.
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}
	
	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity
	 *            Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}
	
	@Override
	public void periodic() {
		processVision();
		motorTab.update();
	}
	
	@Override
	public void simulationPeriodic() {}
	
	private void processVision() {
		visionMeasurement = vision.updateOdometry();
		if (visionMeasurement.isPresent()) {
			swerveDrive.addVisionMeasurement(visionMeasurement.get().estimatedPose.toPose2d(), visionMeasurement.get().timestampSeconds);
		}
	}
	
	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveDriveKinematics} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}
	
	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not
	 * need to be reset when calling this
	 * method. However, if either gyro angle or module position is reset, this must
	 * be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose
	 *            The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}
	
	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by
	 * odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}
	
	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds
	 *            Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}
	
	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory
	 *            The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}
	
	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake
	 *            True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}
	
	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but
	 * facing toward 0.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}
	
	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
	 * the angle of the robot.
	 *
	 * @param xInput
	 *            X joystick input for the robot to move in the X direction.
	 * @param yInput
	 *            Y joystick input for the robot to move in the Y direction.
	 * @param headingX
	 *            X joystick which controls the angle of the robot.
	 * @param headingY
	 *            Y joystick which controls the angle of the robot.
	 * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
		return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(), swerveDrive.getMaximumVelocity());
	}
	
	/**
	 * Gets the current yaw angle of the robot, as reported by the imu. CCW
	 * positive, not wrapped.
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading() {
		return swerveDrive.getOdometryHeading();
	}
	
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double thetaInput) {
		xInput = xInput * swerveDrive.getMaximumVelocity();
		yInput = yInput * swerveDrive.getMaximumVelocity();
		thetaInput = Math.pow(thetaInput, 3) * swerveDrive.getMaximumAngularVelocity();
		
		return swerveDrive.swerveController.getRawTargetSpeeds(xInput, yInput, thetaInput);
	}
	
	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
	 * 90deg.
	 *
	 * @param xInput
	 *            X joystick input for the robot to move in the X direction.
	 * @param yInput
	 *            Y joystick input for the robot to move in the Y direction.
	 * @param angle
	 *            The angle in as a {@link Rotation2d}.
	 * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), swerveDrive.getMaximumVelocity());
	}
	
	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot
	 *
	 * @return A ChassisSpeeds object of the current field-relative velocity
	 */
	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}
	
	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}
	
	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}
	
	public double getMaximumVelocity() {
		return swerveDrive.getMaximumVelocity();
	}
	
	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} fpr the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}
	
	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock() {
		swerveDrive.lockPose();
	}
	
	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getRoll() {
		return swerveDrive.getRoll();
	}

}
