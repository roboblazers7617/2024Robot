// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command nullAuto() {
		return Commands.none();
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}

	// Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
Pose2d targetPose = new Pose2d();

// Since AutoBuilder is configured, we can use it to build pathfinding commands

//TODO I dont know if this makes robot move when called or if I have to do smth else
Command pathfindToPose(Pose2d targetPose, double endVelocity, double rotationDelayDistance) {
	return AutoBuilder.pathfindToPose(targetPose, Constants.AutoConstants.AUTO_CONSTRAINTS, endVelocity, rotationDelayDistance);
}
Command pathfindToPathAndFollow(PathPlannerPath path, double rotationDelayDistance) {
	return AutoBuilder.pathfindThenFollowPath(path, Constants.AutoConstants.AUTO_CONSTRAINTS, rotationDelayDistance);
}


}
