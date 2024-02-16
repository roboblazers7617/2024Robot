package frc.robot.shuffleboard;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * This class is used to create a tab on the shuffleboard that displays information about the motor power and current
 */
public class MotorTab extends ShuffleboardTabBase{
	private static MotorTab instance;

	private final DoublePublisher[] busVoltagePublishers = new DoublePublisher[Constants.NUMBER_OF_MOTORS];
	private final DoublePublisher[] optionCurrentPublishers = new DoublePublisher[Constants.NUMBER_OF_MOTORS];
	private final DoublePublisher[] stickyFaultPublisher = new DoublePublisher[Constants.NUMBER_OF_MOTORS];
	private final CANSparkMax[] motors = new CANSparkMax[Constants.NUMBER_OF_MOTORS];
	private int numberOfMotors = 0;

	private final Alert tooManyMotors = new Alert("Too many motors", AlertType.ERROR);
	
	private MotorTab(){

	}

	/**
	 * This method is used to get the instance of the MotorTab
	 * @return the instance of the MotorTab
	 */
	public static MotorTab getInstance(){
		if(instance == null){
			instance = new MotorTab();
		}
		return instance;
	}

	/**
	 * This method is used to add a motor to the MotorTab
	 * @param newMotors an array of the new motors to be added
	 */
	public void addMotor(CANSparkMax[] newMotors){
		if(newMotors.length + numberOfMotors > Constants.NUMBER_OF_MOTORS){
			tooManyMotors.setText("Too many motors. Increase Constants.NUMBER_OF_MOTORS to: " + (newMotors.length + numberOfMotors));
			tooManyMotors.set(true);
			return;
		}

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable networkTable = inst.getTable("logging/MotorTab");
		for(int i = 0; i < newMotors.length; i++){
			motors[i + numberOfMotors] = newMotors[i];
			busVoltagePublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (i+numberOfMotors) + " Bus Voltage").publish();
			optionCurrentPublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (i+numberOfMotors) + " Total Current").publish();
			stickyFaultPublisher[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (i+numberOfMotors) + " Sticky Faults").publish();
		}
		numberOfMotors += newMotors.length;

	}
	@Override
	public void update() {
		for (int i = 0; i < numberOfMotors; i++) {
			busVoltagePublishers[i].set(motors[i].getBusVoltage());
			optionCurrentPublishers[i].set(motors[i].getOutputCurrent());
			stickyFaultPublisher[i].set(motors[i].getStickyFaults());
		}
	}

	@Override
	public void activateShuffleboard() {
		
	}

	@Override
	public String getNetworkTable() {
		return "MotorTab";
	}
	
	
}
