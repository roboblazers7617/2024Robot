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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.shuffleboard.MotorTab;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Arm extends SubsystemBase {
	// Arm
	/** this is the right arm motor */
	private final CANSparkMax leaderArmMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax followerArmMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	
	private final SparkAbsoluteEncoder armAbsoluteEncoder = leaderArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	
	private final SparkPIDController armPIDController = leaderArmMotor.getPIDController();
	
	private final ArmFeedforward armFeedFoward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
	
	// Elevator
	/** the right motor */
	private final CANSparkMax leaderElevatorMotor = new CANSparkMax(ArmConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
	/** the left motor */
	private final CANSparkMax followerElevatorMotor = new CANSparkMax(ArmConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
	
	/** the potentiometer for the elevator */
	private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(ElevatorConstants.RIGHT_POTIENTIOMETER_PORT, ElevatorConstants.MAX_HEIGHT, 0);
	
	private final PIDController elevatorPIDController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI,
			ElevatorConstants.KD);
	
	private final ElevatorFeedforward elevatorFeedforward;
	
	private final Alert invalidElevatorMove = new Alert("Invalid Elevator Move", AlertType.ERROR);
	
	/** this is used for the position setpoint, in degrees, for setVelocity() */
	private double setpoint;
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
		
		// setup the arm pid controller
		armPIDController.setP(ArmConstants.KP);
		armPIDController.setI(ArmConstants.KI);
		armPIDController.setD(ArmConstants.KD);
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
		


		
		elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
		
		// TODO: (Brandon) The subsystems shouldn't know about the Shuffleboard tabs.
		// Only the Shuffleboard tabs should know about the subsystems
		// TODO: (Brandon) All the info for one subsystem should be on one tab, correct?
		// All on the Arm tab?
		MotorTab.getInstance()
				.addMotor(new CANSparkMax[] { followerArmMotor, leaderArmMotor, followerElevatorMotor, leaderElevatorMotor });
		
		time.reset();
		time.start();
	}
	
	// do something functions
	
	/**
	 * Stops the arm from reaching its target
	 */
	public void stopArm() {
		armPIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
	}
	
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
		
		// if the target is less than the MIN_ABOVE_PASS_ANGLE, make sure the arm is
		// extended, or retracted, if not, do nothing
		
		// if the arm is less than the threshold to go over the bumper
		if (targetDegrees < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (potentiometer.get() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT // and the elevator is not retracted
					&& potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) { // and the elevator is not
																						// extended
				targetDegrees = ArmConstants.MIN_ABOVE_PASS_ANGLE;
			}
		}
		double feedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(targetDegrees), 0);
		
		armPIDController.setReference(targetDegrees, CANSparkMax.ControlType.kPosition, 0, feedFowardValue, ArbFFUnits.kVoltage);
	}
	
	/**
	 * sets the velocity for the arm by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *            the velocity for the arm in degrees per second
	 */
	private void setArmVelocity(double velocityDegreesPerSec) {
		setpoint = setpoint + velocityDegreesPerSec * dt;
		setpoint = Math.min(setpoint, ArmConstants.MAX_ANGLE);
		setpoint = Math.max(setpoint, ArmConstants.MIN_ANGLE);
		if (setpoint < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (potentiometer.get() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT // and the elevator is not retracted
					&& potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) { // and the elevator is not
																						// extended
				setpoint = ArmConstants.MIN_ABOVE_PASS_ANGLE;
			}
		}
		
		armPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0, armFeedFoward.calculate(Units.degreesToRadians(setpoint), 0), ArbFFUnits.kVoltage);
	}
	
	/**
	 * sets the velocity for the arm.
	 * 
	 * @param velocity
	 *            the velocity for the arm in degrees per second
	 * @return a command to set the velocity for the arm
	 */
	public Command setArmVelocityCommand(Supplier<Double> velocity) {
		Command command = new Command() {
			@Override
			public void initialize() {
				setpoint = armAbsoluteEncoder.getPosition();
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
	 * @return if the elevator was successful
	 */
	public boolean setSelevatorTarget(double target) {
		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		if (target > ElevatorConstants.MAX_HEIGHT) {
			target = ElevatorConstants.MAX_HEIGHT;
		}
		// if the target is less than the min height, set the target to the min height
		if (target < ElevatorConstants.MIN_HEIGHT) {
			target = ElevatorConstants.MIN_HEIGHT;
		}
		// if the arm is below the MIN_ABOVE_PASS_ANGLE, abort
		if (armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			invalidElevatorMove.set(true);
			return false;
		}
		invalidElevatorMove.set(false);
		
		double feedFowardValue = elevatorFeedforward.calculate(Units.degreesToRadians(target), 0);
		double pid = elevatorPIDController.calculate(potentiometer.get(), target);
		leaderElevatorMotor.setVoltage(pid + feedFowardValue);
		return true;
	}
	
	@Override
	public void periodic() {
		dt = time.get() - lastTime;
		lastTime = time.get();
		
		if (ArmConstants.KP != armPIDController.getP()) {
			armPIDController.setP(ArmConstants.KP);
		}
		if (ArmConstants.KI != armPIDController.getI()) {
			armPIDController.setI(ArmConstants.KI);
		}
		if (ArmConstants.KD != armPIDController.getD()) {
			armPIDController.setD(ArmConstants.KD);
		}
	}
	
	// get info functions
	public double getRightAbsoluteEncoderPosition() {
		return armAbsoluteEncoder.getPosition();
	}
}
