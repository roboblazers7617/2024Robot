// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.shuffleboard.MotorTab;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

public class Drivetrain extends SubsystemBase {

	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive swerveDrive;
	private final Vision vision;
	private double driverlimitingFactor = OperatorConstants.DEFAULT_DRIVER_LIMITNG_FACTOR;
	private final SysIdRoutine m_sysIdRoutine;

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory Directory of swerve drive config files.
	 */
	public Drivetrain(Vision vision) {
		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
		// objects being created.
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
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
		m_sysIdRoutine = new SysIdRoutine(
				// Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						// Tell SysId how to plumb the driving voltage to the motors.
						(Measure<Voltage> volts) -> {
							SwerveDriveTest.centerModules(swerveDrive);
							swerveDrive.getModules()[0].getDriveMotor().setVoltage(volts.in(Volts));
							swerveDrive.getModules()[1].getDriveMotor().setVoltage(volts.in(Volts));
							swerveDrive.getModules()[2].getDriveMotor().setVoltage(volts.in(Volts));
							swerveDrive.getModules()[3].getDriveMotor().setVoltage(volts.in(Volts));
							// TODO: angle motors to 0
						},
						// Tell SysId how to record a frame of data for each motor on the mechanism
						// being
						// characterized.
						log -> {
							// Record a frame for the left motors. Since these share an encoder, we consider
							// the entire group to be one motor.
							log.motor("drive-leftfront")
									.voltage(
											m_appliedVoltage.mut_replace(
													swerveDrive.getModules()[0].getDriveMotor().getVoltage()
															* RobotController.getBatteryVoltage(),
													Volts))
									.linearPosition(m_distance.mut_replace(
											swerveDrive.getModules()[0].getPosition().distanceMeters, Meters))
									.linearVelocity(
											m_velocity.mut_replace(
													swerveDrive.getModules()[0].getDriveMotor().getVelocity(),
													MetersPerSecond));

							log.motor("drive-rightfront")
									.voltage(
											m_appliedVoltage.mut_replace(
													swerveDrive.getModules()[1].getDriveMotor().getVoltage()
															* RobotController.getBatteryVoltage(),
													Volts))
									.linearPosition(m_distance.mut_replace(
											swerveDrive.getModules()[1].getPosition().distanceMeters, Meters))
									.linearVelocity(
											m_velocity.mut_replace(
													swerveDrive.getModules()[1].getDriveMotor().getVelocity(),
													MetersPerSecond));

							log.motor("drive-leftback")
									.voltage(
											m_appliedVoltage.mut_replace(
													swerveDrive.getModules()[2].getDriveMotor().getVoltage()
															* RobotController.getBatteryVoltage(),
													Volts))
									.linearPosition(m_distance.mut_replace(
											swerveDrive.getModules()[2].getPosition().distanceMeters, Meters))
									.linearVelocity(
											m_velocity.mut_replace(
													swerveDrive.getModules()[2].getDriveMotor().getVelocity(),
													MetersPerSecond));

							log.motor("drive-rightback")
									.voltage(
											m_appliedVoltage.mut_replace(
													swerveDrive.getModules()[3].getDriveMotor().getVoltage()
															* RobotController.getBatteryVoltage(),
													Volts))
									.linearPosition(m_distance.mut_replace(
											swerveDrive.getModules()[3].getPosition().distanceMeters, Meters))
									.linearVelocity(
											m_velocity.mut_replace(
													swerveDrive.getModules()[3].getDriveMotor().getVelocity(),
													MetersPerSecond));
						},
						// Tell SysId to make generated commands require this subsystem, suffix test
						// state in
						// WPILog with this subsystem's name ("drive")
						this));
		swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
													// via angle.

		setupPathPlanner();
		this.vision = vision;
		for (int i = 0; i < 4; i++) {
			MotorTab.getInstance().addMotor(
					new CANSparkMax[] { (CANSparkMax) swerveDrive.getModules()[i].getDriveMotor().getMotor() });
			MotorTab.getInstance().addMotor(
					new CANSparkMax[] { (CANSparkMax) swerveDrive.getModules()[i].getAngleMotor().getMotor() });
		}
	}

	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {
		AutoBuilder.configureHolonomic(
				this::getPose, // Robot pose supplier
				this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
						new PIDConstants(5.7524, 0, 0),
						// Translation PID constants
						new PIDConstants(
								swerveDrive.swerveController.config.headingPIDF.p,
								swerveDrive.swerveController.config.headingPIDF.i,
								swerveDrive.swerveController.config.headingPIDF.d),
						// Rotation PID constants
						4.5,
						// Max module speed, in m/s
						swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
						// Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig()
				// Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
					var alliance = DriverStation.getAlliance();
					return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
				},
				this // Reference to this subsystem to set requirements
		);
	}


	// Create a path following command using AutoBuilder. This will also trigger
	// event markers.

	/**
	 * Command to drive the robot using translative values and heading as a
	 * setpoint.
	 *
	 * @param translationX Translation in the X direction. Cubed for smoother
	 *                     controls.
	 * @param translationY Translation in the Y direction. Cubed for smoother
	 *                     controls.
	 * @param headingX     Heading X to calculate angle of the joystick.
	 * @param headingY     Heading Y to calculate angle of the joystick.
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
			DoubleSupplier headingY) {
		// swerveDrive.setHeadingCorrection(true); // Normally you would want heading
		// correction for this kind of control.
		return run(() -> {
			double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
			double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
			// Make the robot move
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
					headingX.getAsDouble(),
					headingY.getAsDouble(),
					swerveDrive.getOdometryHeading().getRadians(),
					swerveDrive.getMaximumVelocity()));
		});
	}

	/**
	 * Command to drive the robot using translative values and heading as a
	 * setpoint.
	 *
	 * @param translationX Translation in the X direction.
	 * @param translationY Translation in the Y direction.
	 * @param rotation     Rotation as a value between [-1, 1] converted to radians.
	 * @return Drive command.
	 */
	public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
		// swerveDrive.setHeadingCorrection(true); // Normally you would want heading
		// correction for this kind of control.
		return run(() -> {
			// Make the robot move
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
					translationY.getAsDouble(),
					rotation.getAsDouble() * Math.PI,
					swerveDrive.getOdometryHeading().getRadians(),
					swerveDrive.getMaximumVelocity()));
		});
	}

	/**
	 * Command to drive the robot using translative values and heading as angular
	 * velocity.
	 *
	 * @param translationX     Translation in the X direction. Cubed for smoother
	 *                         controls.
	 * @param translationY     Translation in the Y direction. Cubed for smoother
	 *                         controls.
	 * @param angularRotationX Angular velocity of the robot to set. Cubed for
	 *                         smoother controls.
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return run(() -> {
			// Make the robot move
			swerveDrive.drive(
					new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
							Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
					Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
					true,
					false);
		});
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
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

		swerveDrive.drive(translation,
				rotation,
				fieldRelative,
				false); // Open loop is disabled since it shouldn't be used most of the time.

	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}

	@Override
	public void periodic() {
		processVision();
	}

	@Override
	public void simulationPeriodic() {
	}

	private void processVision() {
		Optional<EstimatedRobotPose> visionMeasurement = vision.updateOdometry();
		if (visionMeasurement.isPresent()) {
			swerveDrive.addVisionMeasurement(visionMeasurement.get().estimatedPose.toPose2d(),
					visionMeasurement.get().timestampSeconds);
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
	 * @param initialHolonomicPose The pose to set the odometry to
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
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public void setWheelsForward() {
		swerveDrive.getModules()[0].setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
		swerveDrive.getModules()[1].setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
		swerveDrive.getModules()[2].setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
		swerveDrive.getModules()[3].setDesiredState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but
	 * facing toward 0.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
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
	 * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return swerveDrive.swerveController.getTargetSpeeds(xInput,
				yInput,
				headingX,
				headingY,
				getHeading().getRadians(),
				swerveDrive.getMaximumVelocity());
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
		xInput = Math.pow(xInput, 3) * swerveDrive.getMaximumVelocity();
		yInput = Math.pow(yInput, 3) * swerveDrive.getMaximumVelocity();
		thetaInput = Math.pow(thetaInput, 3) * swerveDrive.getMaximumAngularVelocity();

		return swerveDrive.swerveController.getRawTargetSpeeds(xInput, yInput, thetaInput);

	}

	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle.
	 * Control the robot at an offset of
	 * 90deg.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param angle  The angle in as a {@link Rotation2d}.
	 * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return swerveDrive.swerveController.getTargetSpeeds(xInput,
				yInput,
				angle.getRadians(),
				getHeading().getRadians(),
				swerveDrive.getMaximumVelocity());
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
	 * Add a fake vision reading for testing purposes.
	 */

	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
	private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
	// Mutable holder for unit-safe linear distance values, persisted to avoid
	// reallocation.
	private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
	// Mutable holder for unit-safe linear velocity values, persisted to avoid
	// reallocation.
	private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

	// Create a new SysId routine for characterizing the drive.

	/** Creates a new Drive subsystem. */
	// public SysIdDrive() {
	// // Add the second motors on each side of the drivetrain
	// m_leftMotor.addFollower(new PWMSparkMax(DriveConstants.kLeftMotor2Port));
	// m_rightMotor.addFollower(new PWMSparkMax(DriveConstants.kRightMotor2Port));

	// // We need to invert one side of the drivetrain so that positive voltages
	// // result in both sides moving forward. Depending on how your robot's
	// // gearbox is constructed, you might have to invert the left side instead.
	// m_rightMotor.setInverted(true);

	// // Sets the distance per pulse for the encoders
	// m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
	// m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
	// }

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutine.dynamic(direction);
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
	 * Add a fake vision reading for testing purposes.
	 */
	public void addFakeVisionReading() {
		swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
	}
}