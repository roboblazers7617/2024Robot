// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LED;
import frc.robot.util.AutoCreator;
import frc.robot.util.FieldHelpers;
import frc.robot.util.FieldHelpers.TagLocation;
import frc.robot.subsystems.Head;
import frc.robot.shuffleboard.AutoDriveCommandsTab;
import frc.robot.shuffleboard.ArmTab;
import frc.robot.shuffleboard.ClimberTab;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.LEDTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.SwerveTab;
import frc.robot.shuffleboard.HeadTab;
import frc.robot.subsystems.Arm;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.HapticCommands;
import frc.robot.commands.MechanismCommands;
import frc.robot.commands.drivetrain.LockWheelsState;
import frc.robot.commands.drivetrain.TurnToTag;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final ShuffleboardInfo shuffleboard;
	private final Head head = new Head();
	private final LED led = new LED(SerialPort.Port.kMXP, head);
	private final Arm arm = new Arm();
	private final Climber climber = new Climber();
	private final RobotStatus robotStatus = new RobotStatus(arm, head);
	private final FieldHelpers myFieldHelper = FieldHelpers.getInstance();
	
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverControllerCommands = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private final CommandXboxController operatorControllerCommands = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
	private final XboxController driverController = driverControllerCommands.getHID();
	private final XboxController operatorController = operatorControllerCommands.getHID();
	private double speedMultiplier = SwerveConstants.REGULAR_SPEED;
	// private final Vision vision = new Vision();
	private final Drivetrain drivetrain = new Drivetrain(/* vision */);
	
	private final Command absoluteDrive = drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> processJoystickAngular(driverControllerCommands.getRightX()), () -> processJoystickAngular(driverControllerCommands.getRightY()));
	
	private final Command rotationDrive = drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> processJoystickAngularButFree(driverControllerCommands.getRightX()));

	private final Command rotationDriveFast = drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> processJoystickAngularButFree(driverControllerCommands.getRightX()*SwerveConstants.FAST_TURN_TIME));

	Command driveFieldOrientedDirectAngleSim = drivetrain.simDriveCommand(
        () -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()),
        () -> driverController.getRawAxis(2));
	
	private boolean isClimbMode = false;

	private final AutoCreator autoCreator = new AutoCreator(this, drivetrain, "Autonomous");
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		
		// Configure the trigger bindings
		configureDefaultCommands();
		configureDriverBindings();
		configureOperatorBindings();
		shuffleboard = ShuffleboardInfo.getInstance();
		ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
		// YOUR CODE HERE | | |
		// \/ \/ \/
		tabs.add(new DriverStationTab(drivetrain, robotStatus));
		
		tabs.add(new ArmTab(arm));
		
		tabs.add(new SwerveTab(drivetrain));
		
		tabs.add(new LEDTab(led));
		
		tabs.add(new HeadTab(head));
		
		tabs.add(new ClimberTab(climber));

		tabs.add(new AutoDriveCommandsTab(drivetrain));
		
		// STOP HERE
		shuffleboard.addTabs(tabs);

		


		PointAwayFromLocation(TagLocation.AMP);
	}

	public void configureAutoBuilder(Alliance allianceColor)
	{
		drivetrain.setupPathPlanner(allianceColor);
		NamedCommands.registerCommand("turnToSpeaker", turnToSpeaker());
		NamedCommands.registerCommand("turnTo0", pointAwayFromSpeaker());
		NamedCommands.registerCommand("IntakeGround", MechanismCommands.IntakeGround(arm, head, false));
		NamedCommands.registerCommand("ShootSpeaker", MechanismCommands.AutonomousShoot(arm,head, drivetrain));
		NamedCommands.registerCommand("shootAmp", MechanismCommands.AutonomousShoot(arm, head, ShootingPosition.AMP));
		NamedCommands.registerCommand("Stow", MechanismCommands.AutoStowAndStopIntake(arm, head));
		NamedCommands.registerCommand("TurnUp", turnSideways());
		NamedCommands.registerCommand("StopShooter", head.SpinDownShooter());
		NamedCommands.registerCommand("TurnDown", turnAwayFromAmp());
		NamedCommands.registerCommand("TurnAndShoot", Commands.sequence(turnToSpeaker(),MechanismCommands.AutonomousShoot(arm, head, drivetrain)));
		NamedCommands.registerCommand("variableShoot", MechanismCommands.PrepareShoot(arm, head, ()->drivetrain.getDistanceToSpeaker()).andThen(MechanismCommands.Shoot(arm, head)));
		
		AutoConstants.AUTO_CHOOSER = AutoBuilder.buildAutoChooser("mid start 2 piece");
	
	}
	private void configureDefaultCommands(){

		drivetrain.setDefaultCommand(!RobotBase.isSimulation() ? absoluteDrive : driveFieldOrientedDirectAngleSim);
		arm.setDefaultCommand(arm.ArmDefaultCommand(() -> Math.abs(operatorController.getRightY()) > OperatorConstants.OPERATOR_JOYSTICK_DEADBAND ? -operatorController.getRightY() * ArmConstants.MAX_MANNUAL_ARM_SPEED : 0, () -> Math.abs(operatorController.getLeftY()) > OperatorConstants.OPERATOR_JOYSTICK_DEADBAND ? -operatorController.getLeftY() * ElevatorConstants.MAX_MANUAL_SPEED : 0));

	}

	private void configureDriverBindings(){
		driverControllerCommands.povRight().whileTrue(turnToSpeaker(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX())));
		
		driverControllerCommands.leftBumper()
				.onTrue(new ScheduleCommand(rotationDrive))
				.onFalse(Commands.runOnce(() -> rotationDrive.cancel()));

		driverControllerCommands.leftTrigger()
				.onTrue(new ScheduleCommand(rotationDriveFast))
				.onFalse(Commands.runOnce(() -> rotationDriveFast.cancel()));
				
		driverControllerCommands.rightBumper()
				.onTrue(Commands.runOnce(() -> speedMultiplier = Math.max(.1, speedMultiplier - SwerveConstants.SLOW_SPEED_DECREMENT)))
				.onFalse(Commands.runOnce(() -> speedMultiplier += SwerveConstants.SLOW_SPEED_DECREMENT));
		driverControllerCommands.rightTrigger()
				.onTrue(Commands.runOnce(() -> speedMultiplier = Math.min(1, speedMultiplier + SwerveConstants.FAST_SPEED_INCREMENT)))
				.onFalse(Commands.runOnce(() -> speedMultiplier -= SwerveConstants.FAST_SPEED_INCREMENT));
		
		driverControllerCommands.povLeft()
				.and(() -> checkAllianceColors(Alliance.Red))
				.whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> Math.cos(Units.degreesToRadians(-150)), () -> Math.sin(Units.degreesToRadians(-150))));
		
		driverControllerCommands.povLeft()
				.and(() -> checkAllianceColors(Alliance.Blue))
				.whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> Math.cos(Units.degreesToRadians(150)), () -> Math.sin(Units.degreesToRadians(150))));
		
		driverControllerCommands.povUp().onTrue(Commands.runOnce(() -> speedMultiplier = Math.min(1, speedMultiplier + SwerveConstants.PRECISE_INCREMENT)));
		driverControllerCommands.povDown().onTrue(Commands.runOnce(() -> speedMultiplier = Math.max(.1, speedMultiplier - SwerveConstants.PRECISE_INCREMENT)));
		
		driverControllerCommands.start().onTrue(Commands.runOnce(() -> drivetrain.zeroGyro()));
		driverControllerCommands.back().onTrue(Commands.runOnce(() -> drivetrain.doVisionUpdates(false)));
		
		//TODO: Shoot should not need the position passed in
		//TODO: Rename DBOT to MID_STAGE to be more descriptive
		driverControllerCommands.a()
				.onTrue(MechanismCommands.PrepareShoot(arm, head, ShootingPosition.DBOT)
					.andThen(HapticCommands.RumbleOne(driverController)))
				.onFalse(MechanismCommands.Shoot(arm, head)
					.andThen(HapticCommands.RumbleTwo(operatorController, driverController)));

		//TODO: This can be turned into a drive to source function
		/*driverControllerCommands.b().onTrue(drivetrain.driveToPose(
				new Pose2d(new Translation2d(2.9,4.11), new Rotation2d( Units.degreesToRadians(-60))))
				.andThen(Commands.runOnce(() ->drivetrain.resetLastAngeScalar())));*/
		// driverControllerCommands.b().onTrue(MechanismCommands.AutonomousShoot(arm,head,drivetrain));
		driverControllerCommands.x().whileTrue(turnToSpeaker(() -> processJoystickVelocity(driverController.getLeftY()), () -> processJoystickVelocity(driverController.getLeftX())));
		
		//Turn to source 
		driverControllerCommands.y().whileTrue(drivetrain.driveCommand(() -> processJoystickVelocity(driverControllerCommands.getLeftY()), () -> processJoystickVelocity(driverControllerCommands.getLeftX()), () -> Math.sin(Units.degreesToRadians(150)), () -> Math.cos(Units.degreesToRadians(150))));
	}

	private void configureOperatorBindings(){
		operatorControllerCommands.x().and(() -> !isClimbMode).onTrue(arm.Stow());
		operatorControllerCommands.y().and(() -> !isClimbMode).whileTrue(head.StartOutake()).onFalse(head.StopIntake());
		operatorControllerCommands.a().and(() -> !isClimbMode)
			.onTrue(MechanismCommands.IntakeGround(arm, head, true)
				.andThen(HapticCommands.RumbleTwo(operatorController, driverController))
				.andThen(arm.Stow())
			);
		operatorControllerCommands.b().and(() -> !isClimbMode)
			.onTrue(MechanismCommands.IntakeSource(arm, head)
				.andThen(HapticCommands.RumbleTwo(operatorController, driverController)));
		
		operatorControllerCommands.leftTrigger()
			.onTrue(MechanismCommands.PrepareShoot(arm, head, ShootingPosition.AMP)
				.andThen(HapticCommands.RumbleOne(operatorController)));
;
		operatorControllerCommands.leftBumper()
			.onTrue(MechanismCommands.Shoot(arm, head)
				.andThen(HapticCommands.RumbleTwo(operatorController, driverController)));
		
		operatorControllerCommands.rightTrigger()
			.onTrue(MechanismCommands.PrepareShoot(arm, head, drivetrain::getDistanceToSpeaker)
					.andThen(HapticCommands.RumbleOne(operatorController)))
				.onFalse(MechanismCommands.PrepareShoot(arm, head, drivetrain::getDistanceToSpeaker)
					.andThen(HapticCommands.RumbleOne(operatorController))
					.andThen(MechanismCommands.Shoot(arm, head))
					.andThen(HapticCommands.RumbleTwo(operatorController, driverController))
					.andThen(arm.Stow()));	

		operatorControllerCommands.rightBumper()
			.onTrue(MechanismCommands.PrepareShoot(arm, head, ShootingPosition.SUBWOOFER)
					.andThen(HapticCommands.RumbleOne(operatorController)))
			.onFalse(MechanismCommands.Shoot(arm, head)
					.andThen(HapticCommands.RumbleTwo(operatorController, driverController)));
		
		operatorControllerCommands.povLeft().onTrue(head.StopIntake().andThen(head.SpinDownShooter()));
		operatorControllerCommands.povRight()
			.onTrue(MechanismCommands.PrepareShoot(arm, head, ShootingPosition.PODIUM)
				.andThen(HapticCommands.RumbleOne(operatorController)))
			.onFalse(MechanismCommands.Shoot(arm, head)
				.andThen(HapticCommands.RumbleTwo(operatorController, driverController)));
				
		
		operatorControllerCommands.povUp().onTrue(Commands.runOnce(() -> climber.setSpeed(ClimberConstants.CLIMB_RATE, ClimberConstants.CLIMB_RATE), climber)).onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.povDown().onTrue(Commands.runOnce(() -> climber.setSpeed(-ClimberConstants.CLIMB_RATE, -ClimberConstants.CLIMB_RATE), climber)).onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		
		operatorControllerCommands.x()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(-ClimberConstants.CLIMB_RATE, 0), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.y()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(ClimberConstants.CLIMB_RATE, 0), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.a()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(0, -ClimberConstants.CLIMB_RATE), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		operatorControllerCommands.b()
				.and(() -> isClimbMode)
				.onTrue(Commands.runOnce(() -> climber.setSpeed(0, ClimberConstants.CLIMB_RATE), climber))
				.onFalse(Commands.runOnce(() -> climber.setSpeed(0, 0), climber));
		
		operatorControllerCommands.start().onTrue(head.IntakePiece());
		operatorControllerCommands.back().onTrue(Commands.runOnce(() -> {
			isClimbMode = !isClimbMode;
		}));
	}

	public String outputValues(Supplier<Double> distance, Supplier<Double> armAngle){
		return "distance: " + distance.get() + "\n arm angle: " + armAngle.get();
	}
	
	private boolean checkAllianceColors(Alliance checkAgainst) {
		if (DriverStation.getAlliance().isPresent()) {
			return DriverStation.getAlliance().get() == checkAgainst;
		}
		return false;
	}
	
	private double processJoystickVelocity(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? (-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND)) * speedMultiplier : MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND) * speedMultiplier;
	}
	
	private double processJoystickAngular(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3) : Math.pow(MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3);
	}
	
	private double processJoystickAngularButFree(double joystickInput) {
		return checkAllianceColors(Alliance.Blue) ? Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3) : Math.pow(-MathUtil.applyDeadband(joystickInput, OperatorConstants.DRIVER_JOYSTICK_DEADBAND), 3);
	}

		public void doVisionUpdates(boolean doVisionUpdates){
			drivetrain.doVisionUpdates(doVisionUpdates);
		}
	
	
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return AutoConstants.AUTO_CHOOSER.getSelected();
	}
	
	public void setMotorBrake(boolean isBraked) {
		drivetrain.setMotorBrake(isBraked);
	}
	
	public Command turnToSpeaker() {
		return new ParallelRaceGroup(new TurnToTag(drivetrain, FieldHelpers.TagLocation.SPEAKER.id(), true), Commands.waitSeconds(1));
	}
	
	public Command turnToSpeaker(Supplier<Double> yMovement, Supplier<Double> xMovement) {
		return new TurnToTag(drivetrain, FieldHelpers.TagLocation.SPEAKER.id(), true, yMovement, xMovement);
	}

	public Command turnToSource(Supplier<Double> yMovement, Supplier<Double> xMovement) {

		return new TurnToTag(drivetrain, FieldHelpers.TagLocation.SOURCE_CLOSE.id(), true, yMovement, xMovement);
	
	}
	
	public void teleopInit() {
		// arm.teleopInit();
	
	}
	
	/**
	 * DOES NOT ACTAULLY TURN TO ZERO BE AWARE
	 */
	public Command pointAwayFromSpeaker() {
		
		return new ParallelRaceGroup(PointAwayFromLocation(TagLocation.SPEAKER),Commands.waitSeconds(0.5));
		/* 
		if (checkAllianceColors(Alliance.Red)) {
			return new ParallelRaceGroup(drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(180)),Commands.waitSeconds(0.5));
		}
		return new ParallelRaceGroup(drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(0)),Commands.waitSeconds(1));
		*/
	}

	public Command PointAwayFromLocation(FieldHelpers.TagLocation location){
		return drivetrain.turnToAngleCommand(myFieldHelper.getPose(location).getRotation().rotateBy(new Rotation2d(Units.degreesToRadians(180))));

	}
	public Command turnSideways(){
		if (checkAllianceColors(Alliance.Red)) {
			return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(-90));
		}
		return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(90));
	}

	public Command turnAwayFromAmp(){

		return PointAwayFromLocation(TagLocation.AMP);
		/* 
		if (checkAllianceColors(Alliance.Red)) {
			return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(90));
		}
		return drivetrain.turnToAngleCommand(Rotation2d.fromDegrees(-90));
		*/
	}

	public void StopShooter()
	{
		head.stopShooter();
	}


}