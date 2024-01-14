// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	/** Creates a new Drivetrain. */
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
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {

	}

	public Pose2d getPose2d() {
		return null;

	}

	public void resetPose(Pose2d pose) {

	}

	public ChassisSpeeds getRobotSpeed() {
		return null;

	}

}
