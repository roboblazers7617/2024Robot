// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTag extends Command {
	/** Creates a new TurnToTag. */
	private Supplier<Optional<Transform3d>> tagPoseSupplier;
	private Transform3d tagPose;
	private final Vision vision;
	private final Drivetrain drivetrain;
	private final int tagID;
	private boolean noInitialTagDetection;

	public TurnToTag(Vision vision, Drivetrain drivetrain, int tagID) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
		this.vision = vision;
		this.tagID = tagID;
		this.drivetrain = drivetrain;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		tagPoseSupplier = () -> vision.findTag(tagID);
		noInitialTagDetection = tagPoseSupplier.get().isEmpty();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(tagPoseSupplier.get().isPresent()){
			tagPose = tagPoseSupplier.get().get();	
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return ((noInitialTagDetection) || (false));
	}
}
