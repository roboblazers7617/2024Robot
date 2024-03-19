// package frc.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import frc.robot.Constants.ArmConstants;;

public class ConstantsTest {
	@Test
	public void testArmAnglesAreWithinBounds() {
		// make sure arm angle constants are within the max and min arm angles
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.SOURCE_ANGLE);
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.AMP_ANGLE);
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.FLOOR_PICKUP);
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.STOW_ANGLE);
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.SPEAKER_SUBWOOFER_ANGLE);
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.SPEAKER_PODIUM_ANGLE);
		assertTrue(ArmConstants.MAX_ANGLE > ArmConstants.MIN_ABOVE_PASS_ANGLE);

		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.SOURCE_ANGLE);
		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.AMP_ANGLE);
		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.FLOOR_PICKUP);
		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.STOW_ANGLE);
		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.SPEAKER_SUBWOOFER_ANGLE);
		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.SPEAKER_PODIUM_ANGLE);
		assertTrue(ArmConstants.MIN_ANGLE < ArmConstants.MIN_ABOVE_PASS_ANGLE);
		
	}
}
