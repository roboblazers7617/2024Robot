// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends TrapezoidProfileSubsystem {
	// Right motor and encoder
	// private final CANSparkMax rightMotor = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	// private final AbsoluteEncoder rightAbsoluteEncoder = rightMotor
	// 		.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	// private final SparkPIDController rightPIDController = rightMotor.getPIDController();

	// Left motor and encoder
	private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final AbsoluteEncoder leftAbsoluteEncoder = leftMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController leftPIDController = leftMotor.getPIDController();
	private double currentTarget = -1;
	private double maxCurrentTarget = 0;

	// private final ArmFeedforward feedFoward = new
	// ArmFeedforward(ClimberConstants.KS, ClimberConstants.KG,
	// ClimberConstants.KV);

	/** Creates a new Climber. */
	public Arm() {
		super(new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION),0);
		disable();
		// rightMotor.restoreFactoryDefaults();
		// rightMotor.setIdleMode(IdleMode.kBrake);
		// rightMotor.setSmartCurrentLimit(20);

		leftMotor.restoreFactoryDefaults();
		leftMotor.setIdleMode(IdleMode.kBrake);
		leftMotor.setSmartCurrentLimit(20);

		// rightPIDController.setP(ClimberConstants.KP);
		// rightPIDController.setI(ClimberConstants.KI);
		// rightPIDController.setD(ClimberConstants.KD);
		// rightPIDController.setIZone(ClimberConstants.kIz);
		// rightPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);
		// rightPIDController.setFeedbackDevice(rightAbsoluteEncoder);
		// rightPIDController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionMinOutputVelocity(ClimberConstants.minVel, ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionMaxVelocity(ClimberConstants.MAX_VELOCITY, ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionAllowedClosedLoopError(ClimberConstants.MAX_ERROR, ClimberConstants.SLOT_ID);
		// rightPIDController.setSmartMotionMaxAccel(ClimberConstants.MAX_ACCELERATION, ClimberConstants.SLOT_ID);

		leftPIDController.setP(ArmConstants.KP);
		leftPIDController.setI(ArmConstants.KI);
		leftPIDController.setD(ArmConstants.KD);
		leftPIDController.setIZone(ArmConstants.kIz);
		leftPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		leftPIDController.setFeedbackDevice(leftAbsoluteEncoder);


		// rightAbsoluteEncoder.setPositionConversionFactor(ClimberConstants.ABS_POSITION_CONVERSION_FACTOR);
		// rightAbsoluteEncoder.setVelocityConversionFactor(ClimberConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// // rightAbsoluteEncoder.setZeroOffset(ClimberConstants.OFFSET);
		// rightAbsoluteEncoder.setInverted(false);

		leftAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// leftAbsoluteEncoder.setZeroOffset(ClimberConstants.OFFSET);
		leftAbsoluteEncoder.setInverted(false);

		// TODO make one inverted
	}

	@Override
	public void useState(TrapezoidProfile.State setpoint){
		leftPIDController.setReference(setpoint.position, CANSparkBase.ControlType.kPosition);
		currentTarget = setpoint.position;
		if (setpoint.position > maxCurrentTarget){
			maxCurrentTarget = setpoint.position;
		}
		// System.out.println(setpoint.position);
	}


	// do something functions
	public void raiseArm() {
		enable();
		setTarget(0);
	}

	public void lowerArm() {
		enable();
		setTarget(45);
	}

	public void stopArm(){
		disable();
	}

	private void setTarget(double target) {
		// rightPIDController.setReference(target, CANSparkBase.ControlType.kSmartMotion);
		// leftPIDController.setReference(target, CANSparkBase.ControlType.kSmartMotion);
		// leftPIDController.setReference(target, CANSparkMax.ControlType.kPosition,0,2,
		// feedFoward.calculate(target*(Math.PI/80)));

		setGoal(target);
		
		// setGoal(new TrapezoidProfile.State(target, 0));
	}

	@Override
	public void periodic() {
		super.periodic();
		// This method will be called once per scheduler run
	}

	// get info functions
	public double getRightAbsoluteEncoderPosition() {
		// return rightAbsoluteEncoder.getPosition();
		return -1;
	}

	public double getLeftAbsoluteEncoderPosition() {
		return leftAbsoluteEncoder.getPosition();
	}

	public double getCurrentTarget(){
		return currentTarget;
	}


}

