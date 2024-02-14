// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.shuffleboard.MotorTab;

public class Arm extends SubsystemBase {
	// Right motor and encoder
	private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final SparkAbsoluteEncoder rightAbsoluteEncoder = rightArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController rightArmPIDController = rightArmMotor.getPIDController();

	// Left motor and encoder
	//TODO: (Brandon) One of the arm motors will be the leader and the other the follower.
	// A suggestion (but not required) to make the code easier to read would be to name the motors "leader" and "follower" rather than
	// right and left so a person doesn't need to remember which is the leader and which is the follower
	private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	//TODO: (Brandon) There will only be one Absolute Encoder on the arm
	private final SparkAbsoluteEncoder leftAbsoluteEncoder = leftArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	//TODO: (Brandon) I *think* we only need one PID controller per mechanism becuase the motors will be seutp as leader and follower. 
	// You only need to have one PID controller running
	private final SparkPIDController leftArmPIDController = leftArmMotor.getPIDController();
	private double currentArmTarget = -1;

	private final ArmFeedforward armFeedFoward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG,
			ArmConstants.KV);

	//TODO: (Brandon) One of the arm motors will be the leader and the other the follower.
	// A suggestion (but not required) to make the code easier to read would be to name the motors "leader" and "follower" rather than
	// right and left so a person doesn't need to remember which is the leader and which is the follower

	// elevator motors and encoders
	// right motor
	private final CANSparkMax rightElevatorMotor = new CANSparkMax(ArmConstants.RIGHT_ELEVATOR_MOTOR_ID,
			MotorType.kBrushless);
	private final AnalogInput rightElevatorInput = new AnalogInput(ElevatorConstants.RIGHT_POTIENTIOMETER_PORT);
	private final AnalogPotentiometer rightElevatorPotentiometer = new AnalogPotentiometer(rightElevatorInput,
			ElevatorConstants.MAX_HEIGHT, 0);
	
	private final SparkPIDController rightElevatorPIDController = rightElevatorMotor.getPIDController();

	// left motor
	private final CANSparkMax leftElevatorMotor = new CANSparkMax(ArmConstants.LEFT_ELEVATOR_MOTOR_ID,
			MotorType.kBrushless);
	private final AnalogInput leftElevatorInput = new AnalogInput(ElevatorConstants.LEFT_POTIENTIOMETER_PORT);
	private final AnalogPotentiometer leftElevatorPotentiometer = new AnalogPotentiometer(leftElevatorInput,
			ElevatorConstants.MAX_HEIGHT, 0);
	private final SparkPIDController leftElevatorPIDController = leftElevatorMotor.getPIDController();

	private final ElevatorFeedforward elevatorFeedforward;

	//TODO: (Brandon) Putting as something to talk about so we don't forget... the feedforward values for the elevator may change based on the arm pivot angle,
	// and the feedforward values for the arm may change based on the elevator extension. Need to think through this issue...
	/** Creates a new Arm. */
	public Arm() {

		// setup arm motors

		rightArmMotor.restoreFactoryDefaults();
		rightArmMotor.setIdleMode(IdleMode.kBrake);
		//TODO: (Brandon) the current limit for the arm motors may need to be 40 Amps. Will need to look into this
		rightArmMotor.setSmartCurrentLimit(20);

		leftArmMotor.restoreFactoryDefaults();
		leftArmMotor.setIdleMode(IdleMode.kBrake);
		//TODO: (Brandon) the current limit for the arm motors may need to be 40 Amps. Will need to look into this
		leftArmMotor.setSmartCurrentLimit(20);
		//TODO: (Brandon) You are correct that one of the motors will be inverted. You will need to do an experiment before running
		// the code to see which one it is
 		leftArmMotor.setInverted(true);

		// setup arm pid controllers

		rightArmPIDController.setP(ArmConstants.KP);
		rightArmPIDController.setI(ArmConstants.KI);
		rightArmPIDController.setD(ArmConstants.KD);
		rightArmPIDController.setIZone(ArmConstants.kIz);
		rightArmPIDController.setOutputRange(ArmConstants.kMinOutput,
				ArmConstants.kMaxOutput);
		rightArmPIDController.setFeedbackDevice(rightAbsoluteEncoder);
		rightArmPIDController.setPositionPIDWrappingEnabled(true);

		leftArmPIDController.setP(ArmConstants.KP);
		leftArmPIDController.setI(ArmConstants.KI);
		leftArmPIDController.setD(ArmConstants.KD);
		leftArmPIDController.setIZone(ArmConstants.kIz);
		leftArmPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		leftArmPIDController.setFeedbackDevice(leftAbsoluteEncoder);
		leftArmPIDController.setPositionPIDWrappingEnabled(true);

		// setup arm encoders

		rightAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		rightAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		//TODO: (Brandon) Will need to manually move the arm to see if the absolute encoder needs to be inverted
		rightAbsoluteEncoder.setInverted(false);

		leftAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);

		// setup elevator motors
		rightElevatorMotor.restoreFactoryDefaults();
		rightElevatorMotor.setIdleMode(IdleMode.kBrake);
		//TODO: (Brandon) the current limit for the elevator motors may need to be 40 Amps. Will need to look into this
		rightElevatorMotor.setSmartCurrentLimit(20);

		leftElevatorMotor.restoreFactoryDefaults();
		leftElevatorMotor.setIdleMode(IdleMode.kBrake);
		//TODO: (Brandon) the current limit for the elevator motors may need to be 40 Amps. Will need to look into this
		leftElevatorMotor.setSmartCurrentLimit(20);

		// setup elevator pid controllers
		rightElevatorPIDController.setP(ElevatorConstants.KP);
		rightElevatorPIDController.setI(ElevatorConstants.KI);
		rightElevatorPIDController.setD(ElevatorConstants.KD);
		rightElevatorPIDController.setIZone(ElevatorConstants.kIz);
		rightElevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput,
				ElevatorConstants.kMaxOutput);
		rightElevatorPIDController.setFeedbackDevice(rightElevatorInput);

		leftElevatorPIDController.setP(ElevatorConstants.KP);
		leftElevatorPIDController.setI(ElevatorConstants.KI);
		leftElevatorPIDController.setD(ElevatorConstants.KD);
		leftElevatorPIDController.setIZone(ElevatorConstants.kIz);
		leftElevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput,
				ElevatorConstants.kMaxOutput);
		leftElevatorPIDController.setFeedbackDevice(rightElevatorInput);

		leftAbsoluteEncoder.setInverted(true);

		elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);

		//TODO: (Brandon) The subsystems shouldn't know about the Shuffleboard tabs. Only the Shuffleboard tabs should know about the subsystems
		//TODO: (Brandon) All the info for one subsystem should be on one tab, correct? All on the Arm tab?
		MotorTab.getInstance()
				.addMotor(new CANSparkMax[] { leftArmMotor, rightArmMotor, leftElevatorMotor, rightElevatorMotor });
	}

	// do something functions

	public void stopArm() {
		// disable();
		leftArmPIDController.setReference(leftAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
		rightArmPIDController.setReference(rightAbsoluteEncoder.getPosition(), CANSparkMax.ControlType.kPosition, 0);
	}

	//TODO: (Brandon) What is the measurement of the parameter target? If it is degrees, make that part of the name of the parameter so it is really clear
	public void setArmTarget(double target) {

		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		// height
		if (target > ArmConstants.MAX_ANGLE) {
			target = ArmConstants.MAX_ANGLE;
		}
		// if the target is less than the min height, set the target to the min height
		//TODO: (Brandon) This should be a constant as well as the minimum may not be zero. 
		if (target < 0) {
			target = 0;
		}

		// if the target is less than the MIN_ABOVE_PASS_ANGLE, make sure the arm is
		// extended, or retracted, if not, do nothing
		//TODO: (Brandon) I think I know what you are doing here, but can you walk me through it?
		if (target < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (leftElevatorAbsoluteEncoder.getPosition() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT
					&& leftAbsoluteEncoder.getPosition() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) {
				return;
			}
		}

		//TODO: (Brandon) This can be at a class level. You don't need to create a feedforward each time
		double feedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(target), 0);

		leftArmPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
		rightArmPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);

	}

	//TODO: (Brandon) What are the units of the parameter target? Please add that to the parameter name such as targetInches
	public void setSelevatorTarget(double target){
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
		if (leftAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			return;
		}

		//TODO: (Brandon) This can be at a class level. You don't need to create a feedforward each time
		//TODO: (Brandon) I don't think the units for the feedforward will be radians since it is an elevator? Shouldn't this be something else?

		double feedFowardValue = elevatorFeedforward.calculate(Units.degreesToRadians(target), 0);
		leftElevatorPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
		rightElevatorPIDController.setReference(target, CANSparkMax.ControlType.kPosition, 0,
				feedFowardValue, ArbFFUnits.kVoltage);
	}

	@Override
	public void periodic() {

		if (ArmConstants.KP != leftArmPIDController.getP()) {
			leftArmPIDController.setP(ArmConstants.KP);
		}
		if (ArmConstants.KI != leftArmPIDController.getI()) {
			leftArmPIDController.setI(ArmConstants.KI);
		}
		if (ArmConstants.KD != leftArmPIDController.getD()) {
			leftArmPIDController.setD(ArmConstants.KD);
		}
	}

	// get info functions
	public double getRightAbsoluteEncoderPosition() {
		return rightAbsoluteEncoder.getPosition();
	}

	public double getLeftAbsoluteEncoderPosition() {
		return leftAbsoluteEncoder.getPosition();
	}

	public double getArmCurrentTarget() {
		return currentArmTarget;
	}

}
