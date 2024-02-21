package frc.robot.shuffleboard;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * This class is used to create a tab on the shuffleboard that displays information about the motor power and current
 */
public class MotorTab{
	private final DoublePublisher[] busVoltagePublishers;
	private final DoublePublisher[] optionCurrentPublishers;
	private final DoublePublisher[] stickyFaultPublisher;
	private final CANSparkMax[] motors = new CANSparkMax[Constants.NUMBER_OF_MOTORS];
	private int numberOfMotors = 0;
	private final int totalNumberOfMotors;

	private final Alert tooManyMotors = new Alert("Too many motors", AlertType.ERROR);

	private final String tabName;
	
	public MotorTab(int totalNumberOfMotors, String tabName){
		busVoltagePublishers = new DoublePublisher[totalNumberOfMotors];
		optionCurrentPublishers = new DoublePublisher[totalNumberOfMotors];
		stickyFaultPublisher = new DoublePublisher[totalNumberOfMotors];

		this.tabName = tabName;
		this.totalNumberOfMotors = totalNumberOfMotors;
	}


	/**
	 * This method is used to add a motor to the MotorTab
	 * @param newMotors an array of the new motors to be added
	 */
	public void addMotor(CANSparkMax[] newMotors){
		if(newMotors.length + numberOfMotors > totalNumberOfMotors){
			tooManyMotors.setText("Too many motors. Increase Constants.NUMBER_OF_MOTORS to: " + (newMotors.length + numberOfMotors));
			tooManyMotors.set(true);
			return;
		}

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable networkTable = inst.getTable("logging/" + tabName);
		for(int i = 0; i < newMotors.length; i++){
			motors[i + numberOfMotors] = newMotors[i];
			busVoltagePublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (i+numberOfMotors) + " Bus Voltage").publish();
			optionCurrentPublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (i+numberOfMotors) + " Total Current").publish();
			stickyFaultPublisher[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (i+numberOfMotors) + " Sticky Faults").publish();
		}
		numberOfMotors += newMotors.length;

	}
	/** this MUST be called in periodic() */
	public void update() {
		for (int i = 0; i < numberOfMotors; i++) {
			busVoltagePublishers[i].set(motors[i].getBusVoltage());
			optionCurrentPublishers[i].set(motors[i].getOutputCurrent());
			stickyFaultPublisher[i].set(motors[i].getStickyFaults());
		}
	}

	
	
}
