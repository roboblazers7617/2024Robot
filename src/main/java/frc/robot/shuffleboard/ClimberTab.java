package frc.robot.shuffleboard;


import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

		NetworkTable networkTable = inst.getTable("Shuffleboard/climber");

		 leftMotorPosition = networkTable.getDoubleTopic("Left Motor Position").publish();

		 leftMotorSpeed = networkTable.getDoubleTopic("Left Motor Speed").publish();

		 rightMotorPosition = networkTable.getDoubleTopic("Right Motor Position").publish();

		rightMotorSpeed = networkTable.getDoubleTopic("Right Motor Speed").publish();

		//motorTab.addMotor(climber.getMotors());


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
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("climber");
		tab.add("Left Motor Position", 0.0).withPosition(0, 0);
		tab.add("Left Motor Speed", 0.0).withPosition(0, 1);
		tab.add("Right Motor Position", 0.0).withPosition(1, 0);
		tab.add("Right Motor Speed", 0.0).withPosition(1, 1);
	}

	@Override
	public String getNetworkTable() {
		return "climber";
	}

}
