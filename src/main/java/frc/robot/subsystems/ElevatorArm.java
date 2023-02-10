package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

// file
// this is a cmomment
// this is another cmomment
//Hello world x10000

public class ElevatorArm {

    Rotation2d target_angle = new Rotation2d();
    double target_magnitude = 0.0;

    /**
     * the coordanates are absolute (not relative to arm base)
     * @param cartesian
     */
    private void calculateFromPolar(Translation2d cartesian) {
        target_angle = cartesian.getAngle();
        target_magnitude = cartesian.getDistance(Constants.ElevatorArm.armOffset);
    }
}
