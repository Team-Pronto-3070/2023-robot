package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;

enum Load {
    None,
    Cube,
    Cone,
}

// file
// this is a cmomment
// this is another cmomment
//Hello world x10000
// This is Nick(100% not Kirill)

// okay 
public class ElevatorArmSubsystem extends SubsystemBase {

    double extention = 0.0;
    Rotation2d angle = new Rotation2d();
    Load load = Load.None;


    Rotation2d target_angle = new Rotation2d();
    double target_extention = 0.0;

    /**
     * the coordanates are absolute (not relative to arm base)
     * @param cartesian
     */
    private void calculateFromPolar(Translation2d cartesian) {
        target_angle = cartesian.getAngle();
        target_extention = cartesian.getDistance(Constants.ElevatorArm.armOffset);
    }

    /// Calculates the KG of the vertical drive.
    /// This is not super straight forward as this
    /// changes with the extention of the arm and the load
    private double getVerticalKG() {
        // TODO replace with actual calculation
        return Constants.ElevatorArm.lowerArmWeight + (Constants.ElevatorArm.upperArmWeight * extention);
    }

    private ArmFeedforward getVerticalFeedForward() {
        return new ArmFeedforward(
            Constants.ElevatorArm.VerticalDrive.KS,
            getVerticalKG(),
            Constants.ElevatorArm.VerticalDrive.KV
        );
    }

    public Translation3d getCenterOfMass() {
        double lw = 0.0;
        if (load == Load.Cone) {lw = Constants.ElevatorArm.coneWeight;}
        if (load == Load.Cube) {lw = Constants.ElevatorArm.cubeWeight;}

        Translation2d cm = new Translation2d(((
                (Constants.ElevatorArm.maxExtention / 2) * Constants.ElevatorArm.upperArmWeight) + 
                ((Constants.ElevatorArm.initialArmLength / 2) * Constants.ElevatorArm.lowerArmWeight) + 
                ((extention) * lw)) / 
                (Constants.ElevatorArm.upperArmWeight + Constants.ElevatorArm.lowerArmWeight + lw), angle);
        
        double y = cm.getY();
        double z = cm.getX();

        return new Translation3d( 0.0, y, z);
    }

}
