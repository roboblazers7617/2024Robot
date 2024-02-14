// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class RaiseArm extends Command {
	private final Arm arm;
	private double target;
	private final boolean fast;

	/**
	 * Creates a new RaiseArm.
	 * 
	 * @param arm  arm subsystem
	 * @param fast boolean to determine if the arm's setpoint should be set to the
	 *             end instantly
	 */
	
	 //TODO: (Brandon) I'm not quite sure I understand the logic of this command.
	 // This command could just be a function or a command that is part of the Arm() class
	public RaiseArm(Arm arm, boolean fast) {
		this.arm = arm;
		this.fast = fast;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (fast) {
			arm.setArmTarget(45);
		} else {
			arm.setArmTarget(5);
		}

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (fast) {

		} else {
			target += 0.2;
		}
		arm.setArmTarget(target);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("command has ended");
		// arm.stopArm();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (fast) {
			return target > 45;
		}
		return false;
	}
}
