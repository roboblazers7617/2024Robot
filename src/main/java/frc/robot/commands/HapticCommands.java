package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HapticCommands {
	/**
	 * Trigger rumble on a controller for a specified amount of time.
	 * 
	 * @param controller
	 *            controller to rumble
	 * @param type
	 *            rumble type
	 * @param strength
	 *            rumble strength (0 to 1)
	 * @param length
	 *            rumble time (seconds)
	 */
	public static Command HapticTap(XboxController controller, RumbleType type, double strength, double length) {
		return Commands.runOnce(() -> controller.setRumble(type, strength))
				.andThen(Commands.waitSeconds(length))
				.andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
	}
}
