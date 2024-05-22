package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HapticController {
	private final XboxController controller;
	
	/**
	 * Creates a new HapticController
	 * 
	 * @param controller
	 *                controller
	 */
	public HapticController(XboxController controller) {
		this.controller = controller;
	}
	
	/**
	 * Trigger rumble on the controller for a specified amount of time
	 * Does not trigger in Autonomous
	 *
	 * @param type
	 *                rumble type
	 * @param strength
	 *                rumble strength (0 to 1)
	 * @param length
	 *                rumble time (seconds)
	 */
	public Command HapticTap(RumbleType type, double strength, double length) {
		return Commands.runOnce(() -> controller.setRumble(type, strength))
				.andThen(Commands.waitSeconds(length))
				.finallyDo(() -> controller.setRumble(RumbleType.kBothRumble, 0))
				.unless(() -> DriverStation.isAutonomous())
				.ignoringDisable(true);
	}
}
