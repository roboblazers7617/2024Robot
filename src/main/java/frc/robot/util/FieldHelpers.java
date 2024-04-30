// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldHelpers {

	private AprilTagFieldLayout fieldLayout;
	private static FieldHelpers myInstance = null;

	private FieldHelpers()
	{
		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			fieldLayout = null;
		}
	}

    public static synchronized FieldHelpers getInstance()
    {
        if (myInstance == null)
            myInstance = new FieldHelpers();
 
        return myInstance;
    }

	/* Returns the tag Id of the location based on the current Alliance color */
	public static enum TagLocation {
		AMP(6, 5), 
		SOURCE_FAR(2, 9), 
		SOURCE_CLOSE(1, 10),
		SPEAKER(7, 4);
		
		TagLocation(int blueId, int redId) {
			this.blueId = blueId;
			this.redId = redId;
		}
		
		private final int blueId;
		private final int redId;
		
		public int id ()
		{
			return checkAllianceColors(Alliance.Red) ? blueId : redId;
		}
	}
	
	/* Returns true if is Alliance color matches the checked Alliance.
	 * If no Alliance color is returned from DS, defaults to blue.
	 */
	public static boolean checkAllianceColors(Alliance checkAgainst) {
		if (DriverStation.getAlliance().isPresent()) {
			return DriverStation.getAlliance().get() == checkAgainst;
		}
		return false;
	}
	
	public Pose2d getPose(TagLocation tagLocation){
		return getPose(tagLocation.id());
	}
	
	/*
	 * Returns the pose of an AprilTag. 
	 * NOTE: AprilTags face into the field and thus if we want the rotation of the robot to be where it faces the AprilTag,
	 * the tag pose needs to be rotated by 180. This functions performs that rotation.
	 */
	private Pose2d getPose(int tagId) {
		Pose2d tagPose = fieldLayout.getTagPose(tagId).get().toPose2d();

		// Because the tag faces into the field, we need to rotate the angle by 180 degrees to find what way the robot should face when seeing the tag
		return new Pose2d(tagPose.getTranslation(), tagPose.getRotation()/* .rotateBy(new Rotation2d(Units.degreesToRadians(180)))*/);
	}
}
