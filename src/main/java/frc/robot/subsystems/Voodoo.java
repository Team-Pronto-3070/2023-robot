



package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Voodoo extends SubsystemBase {
    // idk what to put in    

    private Rotation2d desired_lower_arm_rotation = new Rotation2d();
    private Rotation2d desired_upper_arm_rotation = new Rotation2d();

    /**
     * !!! UNFINISHED !!!
     * 
     * calculates the desired roations for the upper and lower
     * segments of the arm in order for the end of the arm
     * to be at the target
     * 
     * @param target (as a Traslation2d)
     * 
     * @return wether the target is reachable or not
     */
    private Boolean tryCalculateDesiredRotation(Translation2d target) {
        
        Translation2d relative_target_pos = target.minus(Constants.VoodooArm.BASE_TRANSLATION_OFFSET);
        
        double dist_from_base_to_target = relative_target_pos.getDistance(new Translation2d());

        double circle_intersect_center_dist = (
            ((Constants.VoodooArm.LOWER_SEGMENT_LEN * Constants.VoodooArm.LOWER_SEGMENT_LEN) - 
            (Constants.VoodooArm.UPPER_SEGMENT_LEN * Constants.VoodooArm.UPPER_SEGMENT_LEN) +
            (dist_from_base_to_target * dist_from_base_to_target)) /
            (2 * dist_from_base_to_target)
        );

        double hypotenuse = Math.sqrt(
            (Constants.VoodooArm.LOWER_SEGMENT_LEN * Constants.VoodooArm.LOWER_SEGMENT_LEN) -
            (circle_intersect_center_dist * circle_intersect_center_dist)
        );

        // use the hypotenuse to calculate the points of intersection
        // of the two circles

        return true;
    }
}
