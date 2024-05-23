package frc.robot.shuffleboard;

/**
 * The base for shuffleboard tabs. Extend this class to create your tab.
 * ALWAYS push your output to
 */
public abstract class ShuffleboardTabBase {
	/**
	 * This will be called every 20ms.
	 */
	public abstract void update();
	
	/**
	 * This will be called when the tab is added to shuffleboard. Prior to this data
	 * can be written to network tables but a shuffleboard tab can never be created.
	 */
	public abstract void activateShuffleboard();
	
	/**
	 * return the name of the network table being pushed to (not including the
	 * logging/)
	 */
	public abstract String getNetworkTable();
}
