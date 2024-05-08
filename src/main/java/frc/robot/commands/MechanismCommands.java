package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShootingConstants;
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
	 * @param arm
	 *            subsystem
	 * @param head
	 *            subsystem
	 * @return Command
	 */
	public static Command IntakeGround(Arm arm, Head head, boolean stopShooter) {
		return 
				Commands.either(head.SpinDownShooter(), Commands.none(), () ->stopShooter)
				.andThen(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece());
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
	public static Command StowStopIntakeAndShooter(Arm arm, Head head) {
		return arm.Stow()
				.andThen(head.StopIntake())
				.andThen(head.SpinDownShooter());
	}

	//THIS ISNT CODE DUPLICATION IT DOES A FUNDAMENTALLY DIFFERENT THING!!!!!
	//TODO: I see that this is different, but when would it be used? Should be renamed to better describe
	//what it does. Why doesn't it stop the shooter? Will it be used for auto?
	public static Command AutoStowAndStopIntake(Arm arm, Head head) {
		return arm.Stow()
				.andThen(head.StopIntake());
	}

	
	/**
	 * will finish when ready
	 * @param arm
	 * @param head
	 * @param position
	 */
	public static Command PrepareShoot(Arm arm, Head head, ShootingPosition position) {
		return arm.SetTargets(position)
				.andThen(head.SpinUpShooter(position));
	}
	
	/**
	 * will finish when ready
	 * @param arm
	 * @param head
	 * @param position
	 */
	public static Command PrepareShoot(Arm arm, Head head, Supplier<Double> distance) {
		return arm.SetTargets(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.VARIABLE_DISTANCE_SHOT));
	}

	public static Command AutonomousPrepareShoot(Arm arm, Head head, Supplier<Double> distance) {
		return arm.SetTargetsAuto(distance)
		.andThen(head.SpinUpShooter(ShootingConstants.AUTO_SHOOT_SPEED));
	}
	
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
	public static Command Shoot(Arm arm, Head head) {
		return Shoot(arm, head, true);
	}

	public static Command Shoot(Arm arm, Head head, boolean stopShooter)
    {
		return Commands.waitUntil(() -> arm.areArmAndElevatorAtTarget())
				.andThen(head.Shoot(stopShooter));
    }

	public static Command AutonomousShoot(Arm arm, Head head, ShootingPosition position){

        return PrepareShoot(arm, head, position)
            .andThen(Shoot(arm, head, false));
    }

    public static Command AutonomousShoot(Arm arm, Head head, Drivetrain drivetrain){

        return AutonomousPrepareShoot(arm, head, () -> drivetrain.getDistanceToSpeaker())
            .andThen(Shoot(arm, head, false));

    }
	
}
