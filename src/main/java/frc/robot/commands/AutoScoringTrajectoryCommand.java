package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PathPlanningUtils;

public class AutoScoringTrajectoryCommand extends DriveToPointCommand {

    private static final Translation2d WALL_SIDE_MIDPOINT = new Translation2d(2.25, 0.9);
    private static final Translation2d LOADING_ZONE_MIDPOINT = new Translation2d(2.25, 4.59);

    private final int scoringSlot;

    /**
     * Constructs an AutoScoringTrajectoryCommand that will create and follow a trajectory 
     * bringing the robot to the given scoring node
     * 
     * @param scoringSlot target slot
     * @param constraints PathConstraints for the trajectory (maximum velocity and maximum acceleration)
     * @param autoBuilder the autobuilder that will be used to follow the trajectory
     * @param swerve the swerve subsystem
     */
    public AutoScoringTrajectoryCommand(int scoringSlot, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        super(List.of(), constraints, autoBuilder, swerve);
        this.scoringSlot = scoringSlot;
    }

    @Override
    public void initialize() {
        Translation2d midPoint = null;

        Translation2d startTranslation = swerve.getPose().getTranslation(); // There might be a slight lag between initialize functions, could set current pose as a variable in super
        double x = startTranslation.getX();
        double y = startTranslation.getY();

        if (x > 2.4 && x < 5) { // NOTE: it is currently jerky when robot starts just above 2.4
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                if ( y > 0 && y < 1.5) {
                    midPoint = WALL_SIDE_MIDPOINT;
                } else if (y > 4 && y < 5) {
                    midPoint = LOADING_ZONE_MIDPOINT;
                }
            } else { // Red
                if (y > Constants.fieldWidthMeters && y < Constants.fieldWidthMeters - 1.5) {
                    midPoint = WALL_SIDE_MIDPOINT;
                } else if (y > Constants.fieldWidthMeters - 4 && y < Constants.fieldWidthMeters - 5) {
                    midPoint = LOADING_ZONE_MIDPOINT;
                }
            }
        }

        Translation2d endTranslation = PathPlanningUtils.getRobotScoringPosition(scoringSlot, DriverStation.getAlliance());

        if (midPoint != null) {
            midPoint = PathPlanningUtils.transformTranslationForAlliance(midPoint, DriverStation.getAlliance());
            points.add(new PathPoint(midPoint, startTranslation.minus(endTranslation).getAngle(), null).withControlLengths(0.25, midPoint.getDistance(endTranslation) / 4));
        }

        Translation2d prevWaypoint = (midPoint == null) ? startTranslation : midPoint;
        points.add(0, new PathPoint(endTranslation, prevWaypoint.minus(endTranslation).getAngle(), new Rotation2d(Math.PI)).withPrevControlLength(endTranslation.getDistance(prevWaypoint) / 2));
        
        super.initialize();
    }
}
