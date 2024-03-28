package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Head;

public class MechanismCommands {
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeSource(Arm arm, Head head) {
		return head.SpinDownShooter()
				.andThen(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeSource(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return IntakeSource(arm, head)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeGround(Arm arm, Head head) {
		return head.SpinDownShooter()
				.andThen(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
	}
	
	/**
	 * will finish after piece has been intaken
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeGround(XboxController driverController, XboxController operatorController, Arm arm, Head head) {
		return IntakeGround(arm, head)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * cancel shoot and intake and stow
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return
	 */
	public static Command Stow(Arm arm, Head head) {
		return arm.Stow()
				.andThen(head.StopIntake())
				.andThen(head.SpinDownShooter());
	}
	//THIS ISNT CODE DUPLICATION IT DOES A FUNDAMENTALLY DIFFERENT THING!!!!!
	//TODO: I see that this is different, but when would it be used? Should be renamed to better describe
	//what it does. Why doesn't it stop the shooter? Will it be used for auto?
	public static Command AutoStow(Arm arm, Head head) {
		return arm.Stow()
				.andThen(head.StopIntake());
	}

	
	/**
	 * will finish when ready
	 * @param arm
	 * @param head
	 * @param position
	 */
	//TODO: Why does this command have a waitUntil the shooter it spun up? That is part of Shoot to 
	//control that logic. Should not be in two places
	public static Command PrepareShoot(Arm arm, Head head, ShootingPosition position) {
		return arm.SetTargets(position)
				.andThen(head.SpinUpShooter(position))
				.andThen(Commands.waitUntil(() -> head.isReadyToShoot()))
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(arm.WaitUntilElevatorAtTarget());
	}
	
	/**
	 * will finish when ready
	 * @param operatorController
	 * @param arm
	 * @param head
	 * @param position
	 * @return
	 */
	//TODO: This duplicates code. PrepareShoot which is called has the wait until the arm and elevator are at target. Should only be in one place. Also should not be checking status of shooter as that is done in Shoot
	public static Command PrepareShoot(XboxController operatorController, Arm arm, Head head, ShootingPosition position) {
		return PrepareShoot(arm, head, position)
				.andThen(Commands.waitUntil(() -> head.isReadyToShoot()))
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}

	//TODO: There needs to be a PrepareShoot() function that takes in a a DoubleSupplier as the distance to calculate and set the positions for arm/elevator/shooter
	/**
	 * will finish after piece has been shot
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param position
	 *            position to shoot from
	 * @return Command
	 */
	//TODO: Shoot should NOT PrepareShoot. Should just shoot
	//TODO: Shoot should not take in the position as doesn't need to know where shooting. That is already set in PrepareShoot. Also Stow() should probably be removed from here and added as an andThen() to the button binding if that should happen
	public static Command Shoot(Arm arm, Head head, ShootingPosition position) {
		return PrepareShoot(arm, head, position)
				.andThen(head.Shoot(position))
				.andThen(arm.Stow());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param position
	 *            position to shoot from
	 * @return Command
	 */
	//TODO: Shoot should not take in the position
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, ShootingPosition position) {
		return Shoot(arm, head, position)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	//TODO: Shoot should not move the arm and elevator
	//TODO: move the arm and elevator set commands to a combined function arm.setTargets(distance)
	//TODO: Why does this call shoot with the SUBWOOFER position?
	public static Command Shoot(Arm arm, Head head, Double distance) {
		return Commands.runOnce(() -> arm.setArmTargetByDistance(distance))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(arm.WaitUntilElevatorAtTarget())
				.andThen(arm.WaitUntilArmAtTarget())
				.andThen(head.Shoot(ShootingPosition.SUBWOOFER));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	//TODO: Shoot should not take in the distance. That is set in the PrepareShoot command
	public static Command Shoot(Arm arm, Head head, Supplier<Double> distance) {
		return Shoot(arm, head, distance.get());
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	//TODO: Shoot should not take in the distance. That is done in PrepareShoot
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, Double distance) {
		return Shoot(arm, head, distance)
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(driverController, RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(HapticCommands.HapticTap(operatorController, RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will wait until finished shooting to finish command
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param distance
	 *            in meters
	 * @return Command
	 */
	//TODO: Shoot should not take in the distance. That is done by the PrepareShoot command
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, Supplier<Double> distance) {
		return Shoot(driverController, operatorController, arm, head, distance.get());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param drivetrain
	 *            subsystem
	 * @return Command
	 */
	//TODO: We don't need this function. the drivetrain.getDistanceToSpeaker function can be passed in as the supplier
	public static Command Shoot(Arm arm, Head head, Drivetrain drivetrain) {
		return Shoot(arm, head, () -> drivetrain.getDistanceToSpeaker());
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @param driverController
	 *            for haptics
	 * @param operatorController
	 *            for haptics
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @param drivetrain
	 *            subsystem
	 * @return Command
	 */
		//TODO: We don't need this function. the drivetrain.getDistanceToSpeaker function can be passed in as the supplier
	public static Command Shoot(XboxController driverController, XboxController operatorController, Arm arm, Head head, Drivetrain drivetrain) {
		return Shoot(driverController, operatorController, arm, head, () -> drivetrain.getDistanceToSpeaker());
	}
}
