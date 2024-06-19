// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;

public class AimAtTag extends Command {
	/** Creates a new AimToTag. */
	private Pose2d tagPose;
	private final Drivetrain drivetrain;
	private final PIDController controller;
	private AprilTagFieldLayout fieldLayout;
	private final int tagID;
	private boolean invertFacing = false;
	private final Supplier<Double> xMovement;
	private final Supplier<Double> yMovement;


	public AimAtTag(Drivetrain drivetrain, int tagID, Supplier<Double> xMovement, Supplier<Double> yMovement) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
		this.drivetrain = drivetrain;
		this.controller = new PIDController(drivetrain.getSwerveController().config.headingPIDF.p, drivetrain.getSwerveController().config.headingPIDF.i, drivetrain.getSwerveController().config.headingPIDF.d);
		controller.setSetpoint(0);
		this.tagID = tagID;
		
		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			fieldLayout = null;
		}
		tagPose = fieldLayout.getTagPose(tagID).get().toPose2d();
		this.xMovement = xMovement;
		this.yMovement = yMovement;
	}

	public AimAtTag(Drivetrain drivetrain, int tagID, boolean invertFacing, Supplier<Double> xMovement, Supplier<Double> yMovement){
		this(drivetrain, tagID, xMovement, yMovement);
		this.invertFacing = invertFacing;
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(invertFacing){
			drivetrain.driveFieldOriented(drivetrain.getTargetSpeeds(xMovement.get(), yMovement.get(), tagPose.getTranslation().minus(drivetrain.getPose().getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180))));	
		}
		else{
		drivetrain.driveFieldOriented(drivetrain.getTargetSpeeds(xMovement.get(), yMovement.get(), tagPose.getTranslation().minus(drivetrain.getPose().getTranslation()).getAngle()));
		
		}
	
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds());
		drivetrain.resetLastAngeScalar();

	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {return false;}
}
