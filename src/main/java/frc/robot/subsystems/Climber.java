// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
	// Right motor and encoder
	private final CANSparkMax rightMotor = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
	private final AbsoluteEncoder rightAbsoluteEncoder = rightMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController rightPIDController = rightMotor.getPIDController();

	// Left motor and encoder
	private final CANSparkMax leftMotor = new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final AbsoluteEncoder leftAbsoluteEncoder = leftMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	private final SparkPIDController leftPIDController = leftMotor.getPIDController();

	// private final ArmFeedforward feedFoward = new ArmFeedforward(ClimberConstants.KS, ClimberConstants.KG,
			// ClimberConstants.KV);

	/** Creates a new Climber. */
	public Climber() {
		rightMotor.restoreFactoryDefaults();
		rightMotor.setIdleMode(IdleMode.kCoast);

		leftMotor.restoreFactoryDefaults();
		leftMotor.setIdleMode(IdleMode.kCoast);

		rightEncoder.setPositionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);
		leftEncoder.setPositionConversionFactor(ClimberConstants.VELOCITY_CONVERSION_FACTOR);

		rightPIDController.setP(ClimberConstants.KP);
		rightPIDController.setI(ClimberConstants.KI);
		rightPIDController.setD(ClimberConstants.KD);

		leftPIDController.setP(ClimberConstants.KP);
		leftPIDController.setI(ClimberConstants.KI);
		leftPIDController.setD(ClimberConstants.KD);

		rightAbsoluteEncoder.setPositionConversionFactor(ClimberConstants.ABS_POSITION_CONVERSION_FACTOR);
		rightAbsoluteEncoder.setVelocityConversionFactor(ClimberConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// rightAbsoluteEncoder.setZeroOffset(ClimberConstants.OFFSET);
		rightAbsoluteEncoder.setInverted(true);

		leftAbsoluteEncoder.setPositionConversionFactor(ClimberConstants.ABS_POSITION_CONVERSION_FACTOR);
		leftAbsoluteEncoder.setVelocityConversionFactor(ClimberConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		// leftAbsoluteEncoder.setZeroOffset(ClimberConstants.OFFSET);
		leftAbsoluteEncoder.setInverted(true);

		// TODO make one inverted
	}

	// do something functions
	public void raiseClimber() {
		setTarget(ClimberConstants.MAX_HEIGHT);
	}

	public void lowerClimber() {
		setTarget(0);
	}
	private void setTarget(double target) {
		rightPIDController.setReference(target, CANSparkBase.ControlType.kPosition);
		leftPIDController.setReference(target, CANSparkBase.ControlType.kPosition);
		// leftPIDController.setReference(target, CANSparkMax.ControlType.kPosition,0,2, feedFoward.calculate(target*(Math.PI/80)));
		
	}

	public void syncEncoders(){
		rightEncoder.setPosition(rightAbsoluteEncoder.getPosition());
		leftEncoder.setPosition(leftAbsoluteEncoder.getPosition());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// get info functions
	public double getRightEncoderPosition() {
		return rightEncoder.getPosition();
	}

	public double getLeftEncoderPosition() {
		return leftEncoder.getPosition();
	}

	public double getRightAbsoluteEncoderPosition() {
		return rightAbsoluteEncoder.getPosition();
	}

	public double getLeftAbsoluteEncoderPosition() {
		return leftAbsoluteEncoder.getPosition();
	}

}
