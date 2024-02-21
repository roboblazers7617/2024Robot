// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.shuffleboard.MotorTab;
import frc.robot.util.TunableNumber;

public class Arm extends SubsystemBase {
	// Arm
	/** this is the right arm motor */
	private final CANSparkMax leaderArmMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax followerArmMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	
	private final SparkAbsoluteEncoder armAbsoluteEncoder = leaderArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	
	private final SparkPIDController armPIDController = leaderArmMotor.getPIDController();
	
	private final TunableNumber extendedArmKP = new TunableNumber("arm", "Extended Arm KP", ArmConstants.KP);
	private final TunableNumber extendedArmKI = new TunableNumber("arm", "Extended Arm KI", ArmConstants.KI);
	private final TunableNumber extendedArmKD = new TunableNumber("arm", "Extended Arm KD", ArmConstants.KD);
	private final TunableNumber retractedArmKP = new TunableNumber("arm", "Retracted Arm KP", ArmConstants.KP);
	private final TunableNumber retractedArmKI = new TunableNumber("arm", "Retracted Arm KI", ArmConstants.KI);
	private final TunableNumber retractedArmKD = new TunableNumber("arm", "Retracted Arm KD", ArmConstants.KD);
	
	private final ArmFeedforward extendedArmFeedForward = new ArmFeedforward(ArmConstants.EXTENDED_KS, ArmConstants.EXTENDED_KG, ArmConstants.EXTENDED_KV);
	private final ArmFeedforward retractedArmFeedForward = new ArmFeedforward(ArmConstants.RETRACTED_KS, ArmConstants.RETRACTED_KG, ArmConstants.RETRACTED_KV);
	
	/** the current target for the arm, it is within the total bounds of the arm but may not be a currently safe move */
	//TODO: (Brandon) This needs to be initialized in the constructor with the current angle
	// of the arm so the arm doesn't try to move on boot-up
	private double armTarget;
	
	// Elevator
	/** the right motor */
	private final CANSparkMax leaderElevatorMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** the left motor */
	private final CANSparkMax followerElevatorMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	
	/** the potentiometer for the elevator */
	private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(ElevatorConstants.RIGHT_POTIENTIOMETER_PORT, ElevatorConstants.MAX_HEIGHT, 0);
	
	private final PIDController elevatorPIDController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
	
	InterpolatingDoubleTreeMap elevatorKSTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKGTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKVTable = new InterpolatingDoubleTreeMap();
	
	private final TunableNumber elevatorKS = new TunableNumber("arm", "Elevator KS", ElevatorConstants.KS);
	private final TunableNumber elevatorKG = new TunableNumber("arm", "Elevator KG", ElevatorConstants.KG);
	private final TunableNumber elevatorKV = new TunableNumber("arm", "Elevator KV", ElevatorConstants.KV);
	
	/** the current target for the elevator, it is within the total bounds of the arm but may not be a currently safe move */
	private double elevatorTarget;
	
	/** this is used for the position setpoint, in degrees, for setVelocity() */
	private double dt, lastTime;
	private Timer time = new Timer();
	
	// TODO: (Brandon) Putting as something to talk about so we don't forget... the
	// feedforward values for the elevator may change based on the arm pivot angle,
	// and the feedforward values for the arm may change based on the elevator
	// extension. Need to think through this issue...
	/** Creates a new Arm. */
	public Arm() {
		// setup arm motors
		
		leaderArmMotor.restoreFactoryDefaults();
		leaderArmMotor.setIdleMode(IdleMode.kBrake);
		leaderArmMotor.setSmartCurrentLimit(ArmConstants.MAX_AMPERAGE);
		
		followerArmMotor.restoreFactoryDefaults();
		followerArmMotor.setIdleMode(IdleMode.kBrake);
		// Will need to look into this
		followerArmMotor.setSmartCurrentLimit(ArmConstants.MAX_AMPERAGE);
		followerArmMotor.setInverted(true);
		followerArmMotor.follow(leaderArmMotor);// TODO: (Brandon) Do I need to tell the follower to invert or is it
												// included in the setInverted method?
		//TODO: (Brandon) Yes, you need to tell it to invert the motor. You can do that with a
		//different version of the follow function that takes two arguments
		
		// setup the arm pid controller
		armPIDController.setP(ArmConstants.KP);
		armPIDController.setI(ArmConstants.KI);
		armPIDController.setD(ArmConstants.KD);
		//TODO: (Brandon) We won't be using a kI value, so we don't need to set the IZone 
		armPIDController.setIZone(ArmConstants.kIz);
		armPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		armPIDController.setFeedbackDevice(armAbsoluteEncoder);
		armPIDController.setPositionPIDWrappingEnabled(true);
		
		// setup the arm encoder
		armAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		armAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// TODO: (Brandon) Will need to manually move the arm to see if the absolute
		// encoder needs to be inverted
		armAbsoluteEncoder.setInverted(false);
		
		// setup elevator motors
		leaderElevatorMotor.restoreFactoryDefaults();
		leaderElevatorMotor.setIdleMode(IdleMode.kBrake);
		leaderElevatorMotor.setSmartCurrentLimit(ElevatorConstants.MAX_AMPERAGE);
		
		followerElevatorMotor.restoreFactoryDefaults();
		followerElevatorMotor.setIdleMode(IdleMode.kBrake);
		followerElevatorMotor.setSmartCurrentLimit(ElevatorConstants.MAX_AMPERAGE);
		followerElevatorMotor.follow(leaderArmMotor);
		
		// TODO: (Brandon) The subsystems shouldn't know about the Shuffleboard tabs.
		// Only the Shuffleboard tabs should know about the subsystems
		// TODO: (Brandon) All the info for one subsystem should be on one tab, correct?
		// All on the Arm tab?
		MotorTab.getInstance()
				.addMotor(new CANSparkMax[] { followerArmMotor, leaderArmMotor, followerElevatorMotor, leaderElevatorMotor });
		
		time.reset();
		time.start();
	}
	
	private ElevatorFeedforward getElevatorFeedforward() {
		// use the interpolation table to get the feedforward values
		// return new ElevatorFeedforward(elevatorKSTable.get(armAbsoluteEncoder.getPosition()), elevatorKGTable.get(armAbsoluteEncoder.getPosition()), elevatorKVTable.get(armAbsoluteEncoder.getPosition()));
		return new ElevatorFeedforward(elevatorKS.get(), elevatorKG.get(), elevatorKV.get());
	}
	
	/** adds feedfoward values to the interpolation table */
	private void addElevatorFeedFowardValues(double ks, double kg, double kv) {
		elevatorKSTable.put(armAbsoluteEncoder.getPosition(), ks);
		elevatorKGTable.put(armAbsoluteEncoder.getPosition(), kg);
		elevatorKVTable.put(armAbsoluteEncoder.getPosition(), kv);
	}
	
	// do something functions
	
	/**
	 * safely set the target angle for the arm
	 * 
	 * @param targetDegrees
	 *            the target angle for the arm in degrees
	 */
	public void setArmTarget(double targetDegrees) {
		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		// height
		targetDegrees = Math.min(targetDegrees, ArmConstants.MAX_ANGLE);
		// if the target is less than the min height, set the target to the min height
		targetDegrees = Math.max(targetDegrees, ArmConstants.MIN_ANGLE);
		
		armTarget = targetDegrees;
	}
	
	/**
	 * raises the arm to the maximum angle
	 * 
	 * @return a command to raise the arm
	 */
	//TODO: (Brandon) This can be re-written to be easier to read as follows in just one line
	// 	return this.runOnce(() -> setArmTarget(ArmConstants.MAX_ANGLE));
	public Command raiseArm() {
		Command command = new Command() {
			@Override
			public void initialize() {
				setArmTarget(ArmConstants.MAX_ANGLE);
			}
			
			@Override
			public boolean isFinished() {
				return armAbsoluteEncoder.getPosition() > ArmConstants.MAX_ANGLE - 1;
			}
		};
		command.addRequirements(this);
		return command;
	}
	
	/**
	 * lowers the arm to the minimum angle
	 * 
	 * @return a command to lower the arm
	 */
	//TODO: (Brandon) This can be re-written to be easier to read as the one above
	//TODO: (Brandon) Do you intend this will lower to the floor? If so, doesn't this command
	//also need to tell the elevator to extend as if it is retracted the arm will not go all the way to the floor.
	//I think the logic is something closer to the stow command where you move both arm and elevator
	public Command lowerArm() {
		Command command = new Command() {
			@Override
			public void initialize() {
				setArmTarget(ArmConstants.MIN_ANGLE);
			}
			
			@Override
			public boolean isFinished() {
				return armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ANGLE + 1;
			}
		};
		command.addRequirements(this);
		return command;
	}
	
	/**
	 * If the elevator is extended and the arm is below the MIN_ABOVE_PASS_ANGLE, raise the arm to the MIN_ABOVE_PASS_ANGLE. After the arm raised, retract the elevator if it is extended. Than lower the arm to the MIN_ANGLE
	 * 
	 * @return a command to stow the arm
	 */
	// TODO: (Brandon) Since you have now added the safety checks into periodic, you don't need to
	// put a bunch of logic here. You can just write a command that orders both the arm and 
	// elevator to move. I think something like this...
	/* 
	public Command stowArm(){
		return this.runOnce(() -> {
				setArmTarget(Constants.ARM_STOW_ANGLE));
				setElevatorTarget(Constants.ELEVATOR_POSITION);
		});
	}
	*/
	public Command stowArm() {
		Command command = new Command() {
			@Override
			public void initialize() {}
			
			@Override
			public void execute() {
				if (potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT && armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
					setArmTarget(ArmConstants.MIN_ABOVE_PASS_ANGLE);
				} else if (armAbsoluteEncoder.getPosition() > ArmConstants.MIN_ABOVE_PASS_ANGLE) {
					setElevatorTarget(ElevatorConstants.MIN_HEIGHT);
				} else if (potentiometer.get() < ElevatorConstants.MAX_BELOW_PASS_HEIGHT) {
					setArmTarget(ArmConstants.MIN_ANGLE);
				}
			}
			
			@Override
			public boolean isFinished() {
				return armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ANGLE + 1 && potentiometer.get() < ElevatorConstants.MAX_BELOW_PASS_HEIGHT + 1;
			}
		};
		command.addRequirements(this);
		return command;
	}
	
	//TODO: (Brandon) This whole funcation can be rewritten to shorten to just one line as 
	/* 
		return this.runOnce(() ->addElevatorFeedFowardValues(elevatorKS.get(), elevatorKG.get(), elevatorKV.get()) );
	*/
	public Command addElevatorFeedFowardValuesCommand() {
		Command command = new Command() {
			@Override
			public void initialize() {
				addElevatorFeedFowardValues(elevatorKS.get(), elevatorKG.get(), elevatorKV.get());
			}
			
			@Override
			public boolean isFinished() {
				return true;
			}
		};
		return command;
	}
	
	/**
	 * sets the velocity for the arm by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *            the velocity for the arm in degrees per second
	 */
	private void setArmVelocity(double velocityDegreesPerSec) {
		armTarget = armTarget + velocityDegreesPerSec * dt;
		armTarget = Math.min(armTarget, ArmConstants.MAX_ANGLE);
		armTarget = Math.max(armTarget, ArmConstants.MIN_ANGLE);
		
	}
	
	/**
	 * sets the velocity for the arm.
	 * 
	 * @param velocity
	 *            the velocity for the arm in degrees per second
	 * @return a command to set the velocity for the arm
	 */
	//TODO: (Brandon) I don't think this logic will work. To create the command I think
	// you just need to do a this.runOnce() with a lambda function calling the setArmVelocity() function. 
	public Command setArmVelocityCommand(Supplier<Double> velocity) {
		Command command = new Command() {
			@Override
			public void initialize() {
				armTarget = armAbsoluteEncoder.getPosition();
			}
			
			@Override
			public void execute() {
				setArmVelocity(velocity.get());
			}
		};
		command.addRequirements(this);
		return command;
	}
	
	/**
	 * safely set the target height for the elevator
	 * 
	 * @param target
	 *            the target height for the elevator in inches
	 */
	public void setElevatorTarget(double target) {
		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		if (target > ElevatorConstants.MAX_HEIGHT) {
			target = ElevatorConstants.MAX_HEIGHT;
		}
		// if the target is less than the min height, set the target to the min height
		if (target < ElevatorConstants.MIN_HEIGHT) {
			target = ElevatorConstants.MIN_HEIGHT;
		}
		elevatorTarget = target;
	}
	
	@Override
	public void periodic() {
		dt = time.get() - lastTime;
		lastTime = time.get();
		
		// if the elevator is extended
		//TODO: (Brandon) Any chance you could change the name of the MAX_BELOW_PASS_HEIGHT
		// constant? My brain (and probably others) don't understand what this value means...
		//TODO: (Brandon) For now, let's try one PID controller and not switch and see if it works.
		// it would simplify things for you
		if (potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) {
			if (extendedArmKP.get() != armPIDController.getP()) {
				armPIDController.setP(ArmConstants.KP);
			}
			if (extendedArmKI.get() != armPIDController.getI()) {
				armPIDController.setI(ArmConstants.KI);
			}
			if (extendedArmKD.get() != armPIDController.getD()) {
				armPIDController.setD(ArmConstants.KD);
			}
		} else {
			if (retractedArmKP.get() != armPIDController.getP()) {
				armPIDController.setP(ArmConstants.KP);
			}
			if (retractedArmKI.get() != armPIDController.getI()) {
				armPIDController.setI(ArmConstants.KI);
			}
			if (retractedArmKD.get() != armPIDController.getD()) {
				armPIDController.setD(ArmConstants.KD);
			}
		}
		// Arm
		
		// current arm target will be the reference set by the PID controller, based on what is currently safe
		double currentArmTarget = armTarget;
		
		// if the arm is less than the threshold to go over the bumper
		if (currentArmTarget < ArmConstants.MIN_ABOVE_PASS_ANGLE) {

			//TODO: (Brandon) I don't quite understand the two checks here. Can you walk me through it?
			if (potentiometer.get() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT // and the elevator is not retracted
					&& potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) { // and the elevator is not
																						// extended
				currentArmTarget = ArmConstants.MIN_ABOVE_PASS_ANGLE;
			}
		}
		
		//TODO: (Brandon) For now, let's try just one feedfoward to see if it works as it simplifies things
		ArmFeedforward armFeedFoward = potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT ? extendedArmFeedForward : retractedArmFeedForward;
		
		double armFeedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(currentArmTarget), 0);
		
		armPIDController.setReference(currentArmTarget, CANSparkMax.ControlType.kPosition, 0, armFeedFowardValue, ArbFFUnits.kVoltage);
		
		// Elevator
		// current elevator target will be the reference set by the PID controller, based on what is currently safe
		double currentElevatorTarget = elevatorTarget;
		// if the arm is less than the threshold to go over the bumper, than the elevator needs to stay on its current side of the bumper
		//TODO:(Brandon) Can you walk me through the two constants? Not sure I understand...
		if (armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (currentElevatorTarget < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT && potentiometer.get() > ElevatorConstants.MIN_ABOVE_PASS_HEIGHT) {
				currentElevatorTarget = ElevatorConstants.MIN_ABOVE_PASS_HEIGHT;
			}
			if (currentElevatorTarget > ElevatorConstants.MAX_BELOW_PASS_HEIGHT && potentiometer.get() < ElevatorConstants.MAX_BELOW_PASS_HEIGHT) {
				currentElevatorTarget = ElevatorConstants.MAX_BELOW_PASS_HEIGHT;
			}
		}
		
		//TODO: (Brandon) Won't the elevator be a linear (Position) value and not and angle?
		double ElevatorFeedFowardValue = getElevatorFeedforward().calculate(Units.degreesToRadians(currentElevatorTarget), 0);
		double pid = elevatorPIDController.calculate(potentiometer.get(), currentElevatorTarget);
		//TODO: (Brandon) We may need to clamp the value being passed to the motor to keep it in a controllable rate 
		// of movement
		leaderElevatorMotor.setVoltage(pid + ElevatorFeedFowardValue);
	}
	
	// get info functions
	public double getRightAbsoluteEncoderPosition() {
		return armAbsoluteEncoder.getPosition();
	}
}
