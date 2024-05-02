// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/** Add your docs here. */
public class Logging {
	private final String field;
	private boolean publishing = false;
	private final DoubleLogEntry log;
	private DoublePublisher publisher;
	
	
	/*
	 * Creates a new Logging object with for specified field and will not automatically push to network tables.
	 * <br>
	 * @param field The root of the logging message.
	 */
	public Logging(String field) {
		this(field, false);
	}
	
	/*
	 * Creates a new Logging object with for specified field and will push to network tables based on alwaysDisplay.
	 * <br>
	 * @param field The root of the logging message.
	 * @param alwaysDisplay Whether the message should always be pushed to network tables.
	 */
	public Logging(String field, boolean alwaysDisplay) {
		this.field = field;
		publishing = alwaysDisplay;

		DataLog logRoot = DataLogManager.getLog();
		log = new DoubleLogEntry(logRoot, field);
		if (alwaysDisplay){
			activateNetworkTables();
		}

	}

	/*
	 * Activates network tables for this logging object.
	 */
	public void activateNetworkTables() {
		publishing = true;
		
		publisher = NetworkTableInstance.getDefault().getDoubleTopic(field).publish();
	}

	/*
	 * Logs the message to the console and network tables if alwaysDisplay is true.
	 * @param message The message to log.
	 */
	public void log(double value) {
		log.append(value);
		if (publishing) {
			publisher.set(value);
		}
	}
}
