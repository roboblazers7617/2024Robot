package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

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
				.finallyDo(() -> controller.setRumble(RumbleType.kBothRumble, 0))
				.ignoringDisable(true);
	}

	public static Command RumbleOne(XboxController controller){
		return new ScheduleCommand(HapticCommands.HapticTap(controller, RumbleType.kBothRumble, 0.3, 0.3));
	}

	public static Command RumbleTwo(XboxController controllerOne, XboxController controllerTwo){
		return new ScheduleCommand(HapticCommands.HapticTap(controllerOne, RumbleType.kBothRumble, 0.3, 0.3))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(controllerTwo, RumbleType.kBothRumble, 0.3, 0.3)));
	}
}
