package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Climber;

public class ClimberTab extends ShuffleboardTabBase {
	private final Climber climber;
	private final DoublePublisher leftMotorPosition;
	private final DoublePublisher leftMotorSpeed;
	private final DoublePublisher rightMotorPosition;
	private final DoublePublisher rightMotorSpeed;
	private final MotorTab motorTab = new MotorTab(2, "climber");
	
	public ClimberTab(Climber climber) {
		this.climber = climber;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("logging/climber");
		
		leftMotorPosition = networkTable.getDoubleTopic("Left Motor Position").publish();
		
		leftMotorSpeed = networkTable.getDoubleTopic("Left Motor Speed").publish();
		
		rightMotorPosition = networkTable.getDoubleTopic("Right Motor Position").publish();
		
		rightMotorSpeed = networkTable.getDoubleTopic("Right Motor Speed").publish();
		
		motorTab.addMotor(climber.getMotors());
	}
	
	@Override
	public void update() {
		leftMotorPosition.set(climber.getPositionLeftMotor());
		leftMotorSpeed.set(climber.getSpeedLeft());
		rightMotorPosition.set(climber.getPositionRightMotor());
		rightMotorSpeed.set(climber.getSpeedRight());
		motorTab.update();
	}
	
	@Override
	public void activateShuffleboard() {}
	
	@Override
	public String getNetworkTable() {
		return "climber";
	}
}
