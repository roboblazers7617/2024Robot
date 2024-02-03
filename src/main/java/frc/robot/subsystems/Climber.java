// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardInfo;

public class Climber extends SubsystemBase {

  
  private final CANSparkMax rightClimber = new CANSparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_PORT, MotorType.kBrushless);
  private final RelativeEncoder rightClimberEncoder;


  private final CANSparkMax leftClimber = new CANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_PORT, MotorType.kBrushless);
  private final RelativeEncoder leftClimberEncoder;


  /**the limit switch for the climber */

  private final double maxEncoderValue;
  /** Creates a new Climber. */
  public Climber() {

    rightClimber.restoreFactoryDefaults();

    leftClimber.restoreFactoryDefaults();
    
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftClimber.setIdleMode(IdleMode.kBrake);

    rightClimber.setInverted(true);

    leftClimberEncoder = leftClimber.getEncoder();
    rightClimberEncoder = rightClimber.getEncoder();
    leftClimberEncoder.setPosition(0.0);
    rightClimberEncoder.setPosition(0.0);

    maxEncoderValue = 130;
  }

  public void setSpeed(double leftSpeed, double rightSpeed){

   // leftClimber.set(speed);
    rightClimber.set(rightSpeed);
    leftClimber.set(leftSpeed);

  }

  

  

  public double getSpeedRight(){
    //  return leftClimber.get();
    return rightClimberEncoder.getVelocity();
  }

  public double getSpeedLeft(){
    //  return leftClimber.get();
    return leftClimberEncoder.getVelocity();
  }

  public void setSpeedLeft(double leftSpeed){
    leftClimber.set(leftSpeed);
  }

  public void setSpeedRight(double rightSpeed){
    rightClimber.set(rightSpeed);
  }
  
  public void resetClimberEncoders()
  {
    leftClimberEncoder.setPosition(0.0);
    rightClimberEncoder.setPosition(0.0);
  }
  

  public double getUpperEncoderLimit(){
    return maxEncoderValue;
  }

  public double getPositionRightMotor(){
    return rightClimberEncoder.getPosition();
  }

  public double getPositionLeftMotor(){
    return leftClimberEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
