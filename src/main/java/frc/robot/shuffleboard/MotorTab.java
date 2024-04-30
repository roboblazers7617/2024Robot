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
public class MotorTab{
	private final DoublePublisher[] busVoltagePublishers;
	private final DoublePublisher[] optionCurrentPublishers;
	private final DoublePublisher[] stickyFaultPublisher;
	private final DoublePublisher[] motorTemperaturePublishers;
	private final DoublePublisher[] motorEncoderPublishers;
	private final CANSparkMax[] motors = new CANSparkMax[Constants.NUMBER_OF_MOTORS];
	private int numberOfMotors = 0;
	private final int totalNumberOfMotors;

	private final Alert tooManyMotors = new Alert("Too many motors", AlertType.ERROR);

	private final String tabName;
	private final ShuffleboardTab tab;
	private final int rowOffset;
	/**
	 * This constructor is used to create a MotorTab, periodic() must be called in the subsystem's periodic method
	 * @param totalNumberOfMotors the total number of motors that will be added to the MotorTab
	 * @param tabName the name of the tab
	 * @param rowOffset the number of rows that all of the motor displays will be lowered by
	 */
	public MotorTab(int totalNumberOfMotors, String tabName, int rowOffset){
		busVoltagePublishers = new DoublePublisher[totalNumberOfMotors];
		optionCurrentPublishers = new DoublePublisher[totalNumberOfMotors];
		stickyFaultPublisher = new DoublePublisher[totalNumberOfMotors];
		motorTemperaturePublishers = new DoublePublisher[totalNumberOfMotors];
		motorEncoderPublishers = new DoublePublisher[totalNumberOfMotors];

		this.tabName = tabName;
		this.totalNumberOfMotors = totalNumberOfMotors;
		tab = Shuffleboard.getTab(tabName);
		this.rowOffset = rowOffset;
	}

	/**
	 * This constructor is used to create a MotorTab, periodic() must be called in the subsystem's periodic method
	 * @param totalNumberOfMotors the total number of motors that will be added to the MotorTab
	 * @param tabName the name of the tab
	 */
	public MotorTab(int totalNumberOfMotors, String tabName){
		this(totalNumberOfMotors, tabName, 0);
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
			busVoltagePublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (newMotors[i].getDeviceId()) + " Bus Voltage").publish();
			optionCurrentPublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (newMotors[i].getDeviceId()) + " Total Current").publish();
			stickyFaultPublisher[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (newMotors[i].getDeviceId()) + " Sticky Faults").publish();
			motorTemperaturePublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (newMotors[i].getDeviceId()) + " Motor Temperature").publish();
			motorEncoderPublishers[i + numberOfMotors] = networkTable.getDoubleTopic("Motor: " + (newMotors[i].getDeviceId()) + " Encoder Position").publish();

			// create shuffleboard entries for each of these with with a position
			
		}
		numberOfMotors += newMotors.length;

	}

	/**
	 * this method must be called in activateShuffleboard() to add the motors to the shuffleboard in the correct position
	 */
	public void activateShuffleboard(){
	/* 	for (int i = 0; i < motors.length; i++) {
			tab.add("Motor: " + (motors[i].getDeviceId()) + " Bus Voltage", busVoltagePublishers[i]).withPosition(0, i + rowOffset);
			tab.add("Motor: " + (motors[i].getDeviceId()) + " Total Current", optionCurrentPublishers[i]).withPosition(1, i + rowOffset);
			tab.add("Motor: " + (motors[i].getDeviceId()) + " Sticky Faults", stickyFaultPublisher[i]).withPosition(2, i + rowOffset);
			tab.add("Motor: " + (motors[i].getDeviceId()) + " Motor Temperature", motorTemperaturePublishers[i]).withPosition(3, i + rowOffset);
			tab.add("Motor: " + (motors[i].getDeviceId()) + " Encoder Position", motorEncoderPublishers[i]).withPosition(4, i + rowOffset);
		}
		*/
	}
	/** this MUST be called in periodic() */
	public void update() {
	/* 	for (int i = 0; i < numberOfMotors; i++) {
			busVoltagePublishers[i].set(motors[i].getBusVoltage());
			optionCurrentPublishers[i].set(motors[i].getOutputCurrent());
			stickyFaultPublisher[i].set(motors[i].getStickyFaults());
			motorTemperaturePublishers[i].set(motors[i].getMotorTemperature());
			motorEncoderPublishers[i].set(motors[i].getEncoder().getPosition());
		}
*/
	}

	
	
}
