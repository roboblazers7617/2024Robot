// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;

public class Head extends SubsystemBase {
	// Shooter
	//TODO: (Max) There will be two motors on the shooter
	private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
	private final SparkPIDController shooterController = shooterMotor.getPIDController();
	private final DigitalInput isNoteInShooter = new DigitalInput(ShooterConstants.SENSOR_DIO);

	/* 
	 * Two motor intake
	 * Always spin bottom motor forward, but reverse top motor to intake from source.
	 */
	private final CANSparkMax intakeMotorBottom = new CANSparkMax(IntakeConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax intakeMotorTop = new CANSparkMax(IntakeConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final DigitalInput isNoteAcquired = new DigitalInput(IntakeConstants.SENSOR_DIO);
	private final DigitalInput isNoteInPosition = new DigitalInput(IntakeConstants.POSITION_SENSOR_DIO);
	
	private boolean readyToShoot = false; // Is the shooter spun up to speed?
	private double shooterSetPoint = 0; // What speed should the shooter be spinning?

	//TODO: (Max) We want to add an Alert that monitors the temperature of the intake motors
	//Last year we had a lot of issues with those motors getting hot, and when they passed a certain temp we needed to let them rest.
	//This year would be good to have an Alert that let us know we need to stop using them

	/** Creates a new Head. */
	public Head() {

		//TODO: (Max) There are other settings for the SparkMax that you will need to set in the constructor
		// the first call should be to restorFactoryDefaults(). Needs to include a current limit, etc. You can look at 
		// last year's code or the code you and Lukas did this fall for a reference

		shooterMotor.setIdleMode(IdleMode.kCoast);

		//TODO: (Max) There are other settings for the SparkMax that you will need to set in the constructor
		// the first call should be to restorFactoryDefaults(). Needs to include a current limit, etc. You can look at 
		// last year's code or the code you and Lukas did this fall for a reference
		//TODO: (Max) Likely one of these motors will be reversed. Just keep this in mind...
		intakeMotorBottom.setIdleMode(IdleMode.kBrake);
		intakeMotorTop.setIdleMode(IdleMode.kBrake);

		//TODO: (Max) You need to setup the PID Controller (kP, kI, etc.) Look at the 2022 robot code for an example
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//TODO: (Max) I don't think we need to call setReference in periodic since we are using a PID controller. You can set it once and then 
		//forget it
		shooterController.setReference(shooterSetPoint, ControlType.kVelocity);
	}

	private void setIntakeBottomSpeed(double intakeSpeed) {
		intakeMotorBottom.set(intakeSpeed);
	}

	private void setIntakeTopSpeed(double intakeSpeed) {
		intakeMotorTop.set(intakeSpeed);
	}

	public Command intakePiece(boolean isFromSource) {

		if (isFromSource) {
			return Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.INTAKE_SPEED); setIntakeTopSpeed(-IntakeConstants.INTAKE_SPEED);}, this)
					.andThen(Commands.waitUntil(() -> isNoteAcquired()))
					.andThen(Commands.runOnce(() -> {setIntakeBottomSpeed(-IntakeConstants.ALIGNMENT_SPEED); setIntakeTopSpeed(-IntakeConstants.ALIGNMENT_SPEED);}))
					.andThen(Commands.waitUntil(() -> isNoteAligned()))
					.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0);});
		} else {
			return Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.INTAKE_SPEED); setIntakeTopSpeed(IntakeConstants.INTAKE_SPEED);}, this)
					.andThen(Commands.waitUntil(() -> isNoteAcquired()))
					.andThen(Commands.runOnce(() -> {setIntakeBottomSpeed(-IntakeConstants.ALIGNMENT_SPEED); setIntakeTopSpeed(-IntakeConstants.ALIGNMENT_SPEED);}))
					.andThen(Commands.waitUntil(() -> isNoteAligned()))
					.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0);});
		}
	}

	public Command outakePiece() {
		return Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.OUTAKE_SPEED); setIntakeTopSpeed(IntakeConstants.OUTAKE_SPEED);}, this)
				//TODO: (Max) I think this should be !isNoteAcquired() to function properly
				.andThen(Commands.waitUntil(() -> isNoteAcquired()))
				//TODO: (Max) You likely will need a short wait command as I believe the beam break that is for if a note is acquired will be between
				// intake and shooter. The intake will need to continue to spin for just a short time after the beam break is clear for the note to be fully out
				.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0);});
	}

	// TODO: (Max) This is where you should call setReference on the shooterMotor
	private void setShooterSpeed(int rpm) {
		shooterSetPoint = rpm;	
	}

	public double getShooterSpeed() {
		return shooterEncoder.getVelocity();
	}

	public double getShooterSetPoint() {
		return shooterSetPoint;
	}

	public Command spinUpShooter(ShooterConstants.ShootingPosition position) {
		//TODO: (Max) I don't think we want to wait in this command until the shooter is at speed as it will block other commands from 
		// executing. This command is simply ordering the shooter to spin.
		return Commands.runOnce(() -> setShooterSpeed(position.rpm()), this)
				.andThen(Commands.waitUntil(() -> isShooterAtSpeed()))
				.finallyDo(() -> readyToShoot = true);
	}

	public Command spinDownShooter() {
		return Commands.runOnce(() -> {readyToShoot = false; setShooterSpeed(ShooterConstants.IDLE_SPEED);}, this)
				//TODO: (Max) I don't think you need a waituntil command here as it will block any other potential commands that need this subsystem from 
				// executing until it is done. We shouldn't care if it is at the IDLE_SPEED or just approaching it
				.andThen(Commands.waitUntil(() -> isShooterAtSpeed()));
	}

	public boolean isShooterAtSpeed() {
		//TODO: (Max) I think this should be multiplying by the VELOCITY_MIN and VELOCITY_MAX as those numbers cannot be a constant number because the shooter
		// velocity will change based on how far the robot is shooting. If you multiply, it can be a percetage of how close to the target
		//TODO: (Max) If you use this to check if ready to shoot, then there needs to be logic that the setPoint it is being commanded to is not the idle setpoint
		return (getShooterSpeed() >= (shooterSetPoint - ShooterConstants.VELOCITY_MINIMUM)) && (getShooterSpeed() <= (shooterSetPoint + ShooterConstants.VELOCITY_MAXIMUM));
	}

	// TODO: (Max) Is there a difference between isShooterAtSpeed and isReadyToShoot? Shouldn't that be the same logic? If this was just used to test the LEDs, you 
	// can now remove it and use the above function.
	public boolean isReadyToShoot() {
		return readyToShoot;
	}

	//TODO: (Max) I think this was just for testing the LEDs, correct? If so then this should be removed or renamed as a test/sim function so someone doesn't actually
	// try to use it
	public void setIsReadyToShoot(boolean isReadyToShoot) {
		readyToShoot = isReadyToShoot;
	}

	public Command shoot() {
		//TODO: (Max) Shouldn't this just call "isShooterAtSpeed"?
		return Commands.waitUntil(() -> isReadyToShoot())
				//TODO: (Max) Could you rename this to something like "FEEDER_SPEED" to more clearly explain the variable?
				.andThen(Commands.runOnce(() -> {setIntakeBottomSpeed(IntakeConstants.SHOOTING_INTAKE_SPEED); setIntakeTopSpeed(IntakeConstants.SHOOTING_INTAKE_SPEED);}))
				.andThen(Commands.waitUntil(() -> !isNoteInShooter()))
				// TODO: (Max) You will probably need a short WaitCommand here to allow the note to fully clear the shooter as the beam break will likely be inbetween the intake and shooter
				.finallyDo(() -> {setIntakeBottomSpeed(0); setIntakeTopSpeed(0); spinDownShooter();});
	}

	public boolean isNoteAcquired() {
		//TODO: (Max) In 2022 we ran into some funkiness where this didn't work. We couldn't figure out why. If this doesn't work, you just need to do this...
		//     boolean sensorVal = shooterSensor.get();
    	//		return !sensorVal;
		return isNoteAcquired.get();
	}

	public boolean isNoteAligned() {
		//TODO: (Max) In 2022 we ran into some funkiness where this didn't work. We couldn't figure out why. If this doesn't work, you just need to do this...
		//     boolean sensorVal = shooterSensor.get();
    	//		return !sensorVal;
		return !isNoteInPosition.get();
	}
	
	public boolean isNoteInShooter() {
		//TODO: (Max) In 2022 we ran into some funkiness where this didn't work. We couldn't figure out why. If this doesn't work, you just need to do this...
		//     boolean sensorVal = shooterSensor.get();
    	//		return !sensorVal;
		return isNoteInShooter.get();
	}
}
