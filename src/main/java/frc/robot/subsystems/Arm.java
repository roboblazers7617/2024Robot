// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.shuffleboard.MotorTab;
// import frc.robot.util.TunableNumber;

public class Arm extends SubsystemBase {
	// Arm
	/** this is the right arm motor */
	private final CANSparkMax leaderArmMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax followerArmMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	
	private final SparkAbsoluteEncoder armAbsoluteEncoder = leaderArmMotor
			.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
	
	private final SparkPIDController armPIDController = leaderArmMotor.getPIDController();
	
	// private final TunableNumber extendedArmKP = new TunableNumber("arm", "Extended Arm KP", ArmConstants.KP);
	// private final TunableNumber extendedArmKI = new TunableNumber("arm", "Extended Arm KI", ArmConstants.KI);
	// private final TunableNumber extendedArmKD = new TunableNumber("arm", "Extended Arm KD", ArmConstants.KD);
	// private final TunableNumber retractedArmKP = new TunableNumber("arm", "Retracted Arm KP", ArmConstants.KP);
	// private final TunableNumber retractedArmKI = new TunableNumber("arm", "Retracted Arm KI", ArmConstants.KI);
	// private final TunableNumber retractedArmKD = new TunableNumber("arm", "Retracted Arm KD", ArmConstants.KD);
	
	private ArmFeedforward extendedArmFeedForward = new ArmFeedforward(ArmConstants.EXTENDED_KS, ArmConstants.EXTENDED_KG, ArmConstants.EXTENDED_KV);
	// private ArmFeedforward retractedArmFeedForward = new ArmFeedforward(ArmConstants.RETRACTED_KS, ArmConstants.RETRACTED_KG, ArmConstants.RETRACTED_KV);
	
	// private final TunableNumber extendedArmKS = new TunableNumber("arm", "Extended Arm KS", ArmConstants.EXTENDED_KS);
	// private final TunableNumber extendedArmKG = new TunableNumber("arm", "Extended Arm KG", ArmConstants.EXTENDED_KG);
	// private final TunableNumber extendedArmKV = new TunableNumber("arm", "Extended Arm KV", ArmConstants.EXTENDED_KV);
	// private final TunableNumber retractedArmKS = new TunableNumber("arm", "Retracted Arm KS", ArmConstants.RETRACTED_KS);
	// private final TunableNumber retractedArmKG = new TunableNumber("arm", "Retracted Arm KG", ArmConstants.RETRACTED_KG);
	// private final TunableNumber retractedArmKV = new TunableNumber("arm", "Retracted Arm KV", ArmConstants.RETRACTED_KV);
	
	/** the current target for the arm, in degrees, it is within the total bounds of the arm but may not be a currently safe move */
	// of the arm so the arm doesn't try to move on boot-up
	private double armTarget;
	/** the last actual arm target */
	private double lastAcutalArmTarget;
	/** arm angle based on distance interpolation table */
	private final InterpolatingDoubleTreeMap armAngleBasedOnDistance = new InterpolatingDoubleTreeMap();
	
	// Elevator
	/** the right motor */
	private final CANSparkMax leaderElevatorMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** the left motor */
	private final CANSparkMax followerElevatorMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	
	private final RelativeEncoder elevatorEncoder = leaderElevatorMotor.getEncoder();
	
	/** the potentiometer for the elevator */
	// private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(ElevatorConstants.RIGHT_POTIENTIOMETER_PORT); // , ElevatorConstants.MAX_HEIGHT, 0
	
	private final SparkPIDController elevatorPIDController = leaderElevatorMotor.getPIDController();
	
	InterpolatingDoubleTreeMap elevatorKSTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKGTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKVTable = new InterpolatingDoubleTreeMap();
	
	// private final TunableNumber elevatorKS = new TunableNumber("arm", "Elevator KS", ElevatorConstants.KS);
	// private final TunableNumber elevatorKG = new TunableNumber("arm", "Elevator KG", ElevatorConstants.KG);
	// private final TunableNumber elevatorKV = new TunableNumber("arm", "Elevator KV", ElevatorConstants.KV);
	
	private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	
	/** the current target for the elevator, it is within the total bounds of the arm but may not be a currently safe move */
	private double elevatorTarget;
	/** the last actual elevator target */
	private double lastAcutalElevatorTarget;
	
	/** this is used for the position setpoint, in degrees, for setVelocity() */
	private double dt, lastTime;
	private Timer time = new Timer();
	
	private final MotorTab motorTab = new MotorTab(4, "arm", 2);
	
	// private final Trigger teleopEnabled;
	
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
		leaderArmMotor.setInverted(true);
		
		followerArmMotor.restoreFactoryDefaults();
		followerArmMotor.setIdleMode(IdleMode.kBrake);
		// Will need to look into this
		followerArmMotor.setSmartCurrentLimit(ArmConstants.MAX_AMPERAGE);
		followerArmMotor.follow(leaderArmMotor, true);
		
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus7, 1000);
		followerArmMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 1000);
		
		// setup the arm pid controller
		armPIDController.setP(ArmConstants.KP);
		armPIDController.setI(ArmConstants.KI);
		armPIDController.setD(ArmConstants.KD);
		armPIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		armPIDController.setFeedbackDevice(armAbsoluteEncoder);
		armPIDController.setPositionPIDWrappingEnabled(false);
		
		// setup the arm encoder
		armAbsoluteEncoder.setPositionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR);
		armAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR);
		armAbsoluteEncoder.setInverted(true);
		armAbsoluteEncoder.setZeroOffset(ArmConstants.ARM_OFFSET);
		
		armTarget = armAbsoluteEncoder.getPosition();
		// lastAcutalArmTarget = armTarget;

		armAngleBasedOnDistance.put(1.27, ArmConstants.SPEAKER_SUBWOOFER_ANGLE);
		armAngleBasedOnDistance.put(2.7, 34.0);
		armAngleBasedOnDistance.put(3.24, 37.0);
		// System.out.println("arm: target: " + armTarget);
		
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus3, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus4, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus5, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus6, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus7, 1000);
		followerElevatorMotor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 1000);
		// setup elevator motors
		leaderElevatorMotor.restoreFactoryDefaults();
		leaderElevatorMotor.setIdleMode(IdleMode.kBrake);
		leaderElevatorMotor.setSmartCurrentLimit(ElevatorConstants.MAX_AMPERAGE);
		leaderElevatorMotor.setInverted(true);
		
		followerElevatorMotor.restoreFactoryDefaults();
		followerElevatorMotor.setIdleMode(IdleMode.kBrake);
		followerElevatorMotor.setSmartCurrentLimit(ElevatorConstants.MAX_AMPERAGE);
		followerElevatorMotor.follow(leaderArmMotor, true);
		
		elevatorTarget = elevatorEncoder.getPosition();
		
		elevatorPIDController.setP(ElevatorConstants.KP);
		elevatorPIDController.setI(ElevatorConstants.KI);
		elevatorPIDController.setD(ElevatorConstants.KD);
		elevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
		// elevatorTarget = potentiometer.get();
		// potentiometer.get()
		
		// TODO: (Brandon) The subsystems shouldn't know about the Shuffleboard tabs.
		// Only the Shuffleboard tabs should know about the subsystems
		// motorTab
		// 		.addMotor(new CANSparkMax[] { followerArmMotor, leaderArmMotor, followerElevatorMotor, leaderElevatorMotor });
		
		time.reset();
		time.start();

		burnFlash();
		
		// teleopEnabled = new Trigger(() -> DriverStation.isTeleopEnabled());
		// teleopEnabled.onTrue(this.runOnce(() -> {
		// elevatorTarget = elevatorEncoder.getPosition();
		// armTarget = armAbsoluteEncoder.getPosition();
		// }));
	}

	private void burnFlash(){
		Timer.delay(0.005);
		leaderArmMotor.burnFlash();
		Timer.delay(0.005);
		followerArmMotor.burnFlash();
		Timer.delay(0.005);
		leaderElevatorMotor.burnFlash();
		Timer.delay(0.005);
		followerElevatorMotor.burnFlash();
		Timer.delay(0.005);
	}
	
	private ElevatorFeedforward getElevatorFeedforward() {
		// use the interpolation table to get the feedforward values
		// return new ElevatorFeedforward(elevatorKSTable.get(armAbsoluteEncoder.getPosition()), elevatorKGTable.get(armAbsoluteEncoder.getPosition()), elevatorKVTable.get(armAbsoluteEncoder.getPosition()));
		return elevatorFeedforward;
	}
	
	private ArmFeedforward getArmFeedforward() {
		// return potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT ? extendedArmFeedForward : retractedArmFeedForward;
		// use just one feedforward for now, if we need 2, use line above
		return extendedArmFeedForward;
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
		targetDegrees = MathUtil.clamp(targetDegrees, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
		
		armTarget = targetDegrees;
	}

	/**
	 * sets the arm target based on the distance to the speaker and the interpolation table
	 * 
	 * @param distance
	 *            the distance to the speaker in meters
	 */
	public void setArmTargetByDistance(double distance) {
		armTarget = MathUtil.clamp(armAngleBasedOnDistance.get(distance), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	/**
	 * raises the arm to the maximum angle
	 * 
	 * @return a command to raise the arm
	 */
	public Command RaiseArm() {
		return this.runOnce(() -> setArmTarget(60));
	}
	
	/**
	 * lowers the arm to the minimum angle
	 * 
	 * @return a command to lower the arm
	 */
	public Command lowerArm() {
		return this.runOnce(() -> setArmTarget(30)); // CHANGE THIS BACK
	}
	
	public Command RaiseElevator() {
		return this.runOnce(() -> setElevatorTarget(ElevatorConstants.MAX_HEIGHT));
	}
	
	public Command lowerElevator() {
		return this.runOnce(() -> setElevatorTarget(ElevatorConstants.MIN_HEIGHT));
	}
	
	/**
	 * stows the arm and elevator
	 * 
	 * @return a command to stow the arm and elevator
	 */
	public Command 
	Stow() {
		return this.runOnce(() -> {
			setArmTarget(ArmConstants.STOW_ANGLE);
			setElevatorTarget(ElevatorConstants.MIN_HEIGHT);
		});
	}
	
	// public Command addElevatorFeedFowardValuesCommand() {
	// 	return this.runOnce(() -> addElevatorFeedFowardValues(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV));
	// }
	
	// public Command generateNewArmFeedFoward() {
	// 	return this.runOnce(() -> {
	// 		extendedArmFeedForward = new ArmFeedforward(ArmConstants.EXTENDED_KS, ArmConstants.EXTENDED_KG, ArmConstants.EXTENDED_KV);
	// 		// retractedArmFeedForward = new ArmFeedforward(retractedArmKS.get(), retractedArmKG.get(), retractedArmKV.get());
	// 	});
	// }
	
	// public Command generateNewElevatorFeedFoward() {
	// 	return this.runOnce(() -> {
	// 		elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	// 	});
	// }
	
	/**
	 * sets the velocity for the arm by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *            the velocity for the arm in degrees per second
	 */
	public void setArmVelocity(double velocityDegreesPerSec) {
		armTarget = armTarget + velocityDegreesPerSec * dt;
		armTarget = MathUtil.clamp(armTarget, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
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
	
	/**
	 * sets the velocity for the elevator by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *            the velocity for the elevator in degrees per second
	 */
	public void setElevatorVelocity(double velocityDegreesPerSec) {
		elevatorTarget = elevatorTarget + velocityDegreesPerSec * dt;
		elevatorTarget = MathUtil.clamp(elevatorTarget, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
		// System.out.println("elevator target in volocity: " + elevatorTarget);
	}
	
	public Command ArmDefaultCommand(Supplier<Double> armVelocity, Supplier<Double> elevatorVelocity) {
		Command command = new RunCommand(() -> {
			setArmVelocity(armVelocity.get());
			setElevatorVelocity(elevatorVelocity.get());
		});
		command.addRequirements(this);
		
		return command;
	}
	
	public Command ToggleBrakeModes() {
		return this.runOnce(() -> {
			if (leaderArmMotor.getIdleMode() == IdleMode.kBrake) {
				leaderArmMotor.setIdleMode(IdleMode.kCoast);
				followerArmMotor.setIdleMode(IdleMode.kCoast);
			} else {
				leaderArmMotor.setIdleMode(IdleMode.kBrake);
				followerArmMotor.setIdleMode(IdleMode.kBrake);
			}
			if (leaderElevatorMotor.getIdleMode() == IdleMode.kBrake) {
				leaderElevatorMotor.setIdleMode(IdleMode.kCoast);
				followerElevatorMotor.setIdleMode(IdleMode.kCoast);
			} else {
				leaderElevatorMotor.setIdleMode(IdleMode.kBrake);
				followerElevatorMotor.setIdleMode(IdleMode.kBrake);
			}
		}).ignoringDisable(true);
	}
	
	@Override
	public void periodic() {
		dt = time.get() - lastTime;
		lastTime = time.get();
		
		// if the elevator is extended
		// TODO: (Brandon) Any chance you could change the name of the MAX_BELOW_PASS_HEIGHT
		// constant? My brain (and probably others) don't understand what this value means...
		// TODO: (Brandon) For now, let's try one PID controller and not switch and see if it works.
		// it would simplify things for you
		// if (potentiometer.get() > ElevatorConstants.MAX_BELOW_PASS_HEIGHT) {
		// if (extendedArmKP.get() != armPIDController.getP()) {
		// armPIDController.setP(extendedArmKP.get());
		// }
		// if (extendedArmKI.get() != armPIDController.getI()) {
		// armPIDController.setI(extendedArmKI.get());
		// }
		// if (extendedArmKD.get() != armPIDController.getD()) {
		// armPIDController.setD(extendedArmKD.get());
		// }
		// }
		// else {
		// if (retractedArmKP.get() != armPIDController.getP()) {
		// armPIDController.setP(ArmConstants.KP);
		// }
		// if (retractedArmKI.get() != armPIDController.getI()) {
		// armPIDController.setI(ArmConstants.KI);
		// }
		// if (retractedArmKD.get() != armPIDController.getD()) {
		// armPIDController.setD(ArmConstants.KD);
		// }
		// }
		
		// Arm
		
		// current arm target will be the reference set by the PID controller, based on what is currently safe
		double currentArmTarget = armTarget;
		
		// if the arm is less than the threshold to go over the bumper
		if (currentArmTarget < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (elevatorEncoder.getPosition() < 18.0) { // and the elevator is not
				// extended
				currentArmTarget = ArmConstants.MIN_ABOVE_PASS_ANGLE;
			}
		}
		if (lastAcutalArmTarget != currentArmTarget) {
			ArmFeedforward armFeedFoward = getArmFeedforward();
			double velocity = 0;
			if (Math.abs(currentArmTarget - armAbsoluteEncoder.getPosition()) > 2){
				velocity = armAbsoluteEncoder.getVelocity();
			}
			double armFeedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(currentArmTarget), velocity);
			// System.out.println("arm feed foward: " + armFeedFowardValue);
			
			armPIDController.setReference(currentArmTarget, CANSparkMax.ControlType.kPosition, 0, armFeedFowardValue, ArbFFUnits.kVoltage);
			lastAcutalArmTarget = currentArmTarget;
		}
		if (!ElevatorConstants.KILL_IT_ALL) {
			// Elevator
			// current elevator target will be the reference set by the PID controller, based on what is currently safe
			double currentElevatorTarget = elevatorTarget;
			// if the arm is less than the threshold to go over the bumper, than the elevator needs to stay on its current side of the bumper
			// TODO:(Brandon) Can you walk me through the two constants? Not sure I understand...
			if (armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
				if (currentElevatorTarget < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT && elevatorEncoder.getPosition() > ElevatorConstants.MIN_ABOVE_PASS_HEIGHT) {
					currentElevatorTarget = ElevatorConstants.MIN_ABOVE_PASS_HEIGHT;
				}
			}
			
			/** this is a constant increase to make the elvator go faster */
			if (currentElevatorTarget != lastAcutalElevatorTarget) {
				double speedyElevatorFeedForward;
				if (Math.abs(currentElevatorTarget - elevatorEncoder.getPosition()) > 5.0) {
					speedyElevatorFeedForward = Math.copySign(2.0, (currentElevatorTarget - elevatorEncoder.getPosition()));
				} else {
					speedyElevatorFeedForward = 0.0;
				}
				
				double elevatorFeedFowardValue = getElevatorFeedforward().calculate(elevatorEncoder.getVelocity());
				elevatorPIDController.setReference(currentElevatorTarget, CANSparkMax.ControlType.kPosition, 0, /*speedyElevatorFeedForward*/ + elevatorFeedFowardValue, ArbFFUnits.kVoltage);
				lastAcutalElevatorTarget = currentElevatorTarget;
			}
			
			motorTab.update();
		}
	}
	
	// get info functions
	public double getArmAbsoluteEncoderPosition() {
		return armAbsoluteEncoder.getPosition();
	}
	
	public double getElevatorAbsoluteEncoderPosition() {
		return elevatorEncoder.getPosition();
	}

	public MotorTab getMotorTab(){
		return motorTab;
	}

	public Command WaitUntilArmAtTarget(){
		return new Command() {
			@Override
			public boolean isFinished() {
				return Math.abs(armTarget - armAbsoluteEncoder.getPosition()) < 5;
			}
		};
	}

	public Command WaitUntilElevatorAtTarget(){
		return new Command() {
			@Override
			public boolean isFinished() {
				return Math.abs(elevatorTarget - elevatorEncoder.getPosition()) < 2;
			}
		};
	}
}
