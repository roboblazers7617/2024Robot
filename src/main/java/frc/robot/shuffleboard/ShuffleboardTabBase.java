package frc.robot.shuffleboard;
/**
 * 
 */
public abstract class ShuffleboardTabBase {
    
	/**
	 * This will be called every 20ms. 
	 */
    public abstract void update();
	/**
	 * This will be called when the tab is added to shuffleboard. Prior to this data can be written to network tables but a shuffleboard tab can never be created.
	 */
	public abstract void activateShuffleboard();
}
