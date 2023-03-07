//the acceleration limiting logic is mostly from team 95
//source: https://github.com/first95/FRC2023/blob/main/2023robot/src/main/java/frc/robot/drivebase/AbsoluteDrive.java

package frc.robot.util;

import static frc.robot.Constants.MassProperties.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.GameObject;

public class AntiTipper {
    private final Supplier<Rotation2d> armAngleSupplier;
    private final DoubleSupplier elevatorPositionSupplier;
    private final Supplier<GameObject> intakeContents;

    // armAngleSupplier should return radians from horizontal
    // elevatorPositionSupplier should return meters from fully retracted
    public AntiTipper(Supplier<Rotation2d> armAngleSupplier, DoubleSupplier elevatorPositionSupplier, Supplier<GameObject> intakeContents) {
        this.armAngleSupplier = armAngleSupplier;
        this.elevatorPositionSupplier = elevatorPositionSupplier;
        this.intakeContents = intakeContents;
    }

    /**
     * Calculates the maximum acceleration allowed in a direction without tipping the robot.
     * @param angle The direction in which to calculate max acceleration, as a Rotation2d.
     *              Note that this is robot-relative.
     */
    private double calcMaxAccel(Rotation2d angle) {
        Translation3d robotCG = calculateOverallCG();
        Translation2d horizontalCG = robotCG.toTranslation2d();

        Translation2d projectedHorizontalCg = new Translation2d(
                (angle.getSin() * angle.getCos() * horizontalCG.getY())
                        + (Math.pow(angle.getCos(), 2) * horizontalCG.getX()),
                (angle.getSin() * angle.getCos() * horizontalCG.getX())
                        + (Math.pow(angle.getSin(), 2) * horizontalCG.getY()));

        // Projects the edge of the wheelbase onto the direction line. Assumes the
        // wheelbase is rectangular.
        // Because a line is being projected, rather than a point, one of the
        // coordinates of the projected point is
        // already known.
        Translation2d projectedWheelbaseEdge;
        double angDeg = angle.getDegrees();
        if (angDeg <= 45 && angDeg >= -45) {
            projectedWheelbaseEdge = new Translation2d(
                    (Constants.Swerve.wheelBase / 2.0),
                    (Constants.Swerve.wheelBase / 2.0) * angle.getTan());
        } else if (135 >= angDeg && angDeg > 45) {
            projectedWheelbaseEdge = new Translation2d(
                    (Constants.Swerve.trackWidth / 2.0) / angle.getTan(),
                    (Constants.Swerve.trackWidth / 2.0));
        } else if (-135 <= angDeg && angDeg < -45) {
            projectedWheelbaseEdge = new Translation2d(
                    (-Constants.Swerve.trackWidth / 2.0) / angle.getTan(),
                    (-Constants.Swerve.trackWidth / 2.0));
        } else {
            projectedWheelbaseEdge = new Translation2d(
                    (-Constants.Swerve.wheelBase / 2.0),
                    (-Constants.Swerve.wheelBase / 2.0) * angle.getTan());
        }

        double horizontalDistance = projectedHorizontalCg.plus(projectedWheelbaseEdge).getNorm();
        double maxAccel = 9.81 * horizontalDistance / robotCG.getZ();
        
        SmartDashboard.putNumber("calcMaxAccel", maxAccel);
        return maxAccel;
    }

    /**
     * Limits a commanded velocity to prevent exceeding the maximum acceleration given by
     * {@link AntiTipper#calcMaxAccel(Rotation2d)}. Note that this takes and returns field-relative velocities.
     * 
     * @param currentAngle The robot's current angle
     * @param currentVelocity The robot's current field-relative velocity
     * @param commandedVelocity The desired velocity
     * @return The limited velocity. This is either the commanded velocity, if attainable, or the closest attainable velocity.
     */
    public Translation2d limitVelocity(Rotation2d currentAngle, Translation2d currentVelocity, Translation2d commandedVelocity) {
        SmartDashboard.putNumber("currentVelocity", currentVelocity.getX());

        // Calculate the commanded change in velocity by subtracting current velocity
        // from commanded velocity
        Translation2d deltaV = commandedVelocity.minus(currentVelocity);
        SmartDashboard.putNumber("deltaV", deltaV.getX());

        // Creates an acceleration vector with the direction of delta V and a magnitude
        // of the maximum allowed acceleration in that direction
        Translation2d maxAccel = new Translation2d(
                calcMaxAccel(deltaV
                        // Rotates the velocity vector to convert from field-relative to robot-relative
                        .rotateBy(currentAngle.unaryMinus())
                        .getAngle()),
                deltaV.getAngle());

        // Calculate the maximum achievable velocity by the next loop cycle.
        // delta V = Vf - Vi = at
        Translation2d maxAchievableDeltaVelocity = maxAccel.times(0.02);

        if (deltaV.getNorm() > maxAchievableDeltaVelocity.getNorm()) {
            return maxAchievableDeltaVelocity.plus(currentVelocity);
        } else {
            // If the commanded velocity is attainable, use that.
            return commandedVelocity;
        }
    }

    private Translation3d calculateOverallCG() {
        double gameObjectMass = intakeContents.get().getMass();

        //horizontal, relative where it would be if it was fully retracted
        Translation3d elevatorCarriageAndGameObject = addCG(elevatorCarriageCG, elevatorCarriageMass, gameObjectLocation, gameObjectMass);

        //horizontal, relative to pivot point
        Translation3d elevatorCarriageCG = new Translation3d(
            elevatorCarriageAndGameObject.getX() + elevatorPositionSupplier.getAsDouble(),
            elevatorCarriageAndGameObject.getY(),
            elevatorCarriageAndGameObject.getZ());
        
        //horizontal, relative to pivot point
        Translation3d overallElevatorCG = addCG(elevatorCarriageCG, elevatorCarriageMass + gameObjectMass, elevatorStaticCG, elevatorStaticMass);

        //relative to pivot point
        Translation3d armCG = overallElevatorCG.rotateBy(new Rotation3d(0.0, -armAngleSupplier.get().getRadians(), 0.0));

        return addCG(armCG.plus(pivotLocation), elevatorCarriageMass + elevatorStaticMass + gameObjectMass, armCG, gameObjectMass);
    }

    private static Translation3d addCG(Translation3d CG1, double mass1, Translation3d CG2, double mass2) {
        return new Translation3d(
            (CG1.getX() * mass1) + (CG2.getX() * mass2),
            (CG1.getY() * mass1) + (CG2.getY() * mass2),
            (CG1.getZ() * mass1) + (CG2.getZ() * mass2)
        ).div(mass1 + mass2);
    }
    
}
