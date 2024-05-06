// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

/** Add your docs here. */
public class Logging {
	private final String field;
	private final String table;
	private boolean publishing = false;
	private final DoubleLogEntry log;
	private DoublePublisher publisher;
	
	/*
	 * Creates a new Logging object with for specified field and will not automatically push to network tables.
	 * <br>
	 * @param field The root of the logging message.
	 */
	public Logging(String table, String field) {
		this(table, field, false);
	}
	
	/*
	 * Creates a new Logging object for specified field and will push to network tables based on alwaysDisplay.
	 * <br>
	 * @param field The root of the logging message.
	 * @param alwaysDisplay Whether the message should always be pushed to network tables.
	 */
	public Logging(String table, String field, boolean alwaysDisplay) {
		this.field = field;
		this.table = table;
		// System.out.println("field: " + field);
		publishing = alwaysDisplay || Constants.debugMode;
		
		DataLog logRoot = DataLogManager.getLog();
		log = new DoubleLogEntry(logRoot, table + "/" + field);
		if (publishing) {
			activateNetworkTables();
		}
	}
	
	/*
	 * Activates network tables for this logging object.
	 */
	private void activateNetworkTables() {
		// publishing = true;
		
		publisher = NetworkTableInstance.getDefault().getTable("Shuffleboard/" + table).getDoubleTopic(field).publish();
		// System.out.println("activated network tables: " + field);
		// DoubleSubscriber subscriber = new DoubleTopic(field).subscribe(0, null);
	}
	
	/*
	 * Logs the message to the console and network tables if alwaysDisplay is true.
	 * @param message The message to log.
	 */
	public void log(double value) {
		// System.out.println("logging");
		log.append(value);
		
		if (!publishing && Constants.debugMode) {
			activateNetworkTables();
			publishing = true;
		}
		
		if (publishing) {
			publisher.set(value);
			// System.out.println("")
		}
	}
}