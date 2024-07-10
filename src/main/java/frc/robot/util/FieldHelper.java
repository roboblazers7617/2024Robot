package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.lang.Exception;
import java.util.Optional;

public class FieldHelper {
	private static AprilTagFieldLayout fieldLayout;
	
	enum TagLocations {
		SOURCE_CLOSE(1, 10), SOURCE_FAR(2, 9), SPEAKER(7, 4), AMP(6, 5);
		
		private final int blueId;
		private final int redId;
		
		TagLocations(int blueId, int redId) {
			this.blueId = blueId;
			this.redId = redId;
		}
		
		public Optional<Pose3d> getPose() {
			if (fieldLayout == null) {
				try {
					fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
				} catch (Exception e) {
					fieldLayout = null;
					return Optional.empty();
				}
			}
			try{
			if(DriverStation.getAlliance().orElseThrow() == Alliance.Blue){
				return fieldLayout.getTagPose(blueId);
			}
			else{
				return fieldLayout.getTagPose(redId);
			}}
			catch(Exception e){
				return Optional.empty();
			}
		}
	}

	public static boolean checkAllianceColors(Alliance checkAgainst) {
		if (DriverStation.getAlliance().isPresent()) {
			return DriverStation.getAlliance().orElseThrow() == checkAgainst;
		}
		return false;
	}
}
