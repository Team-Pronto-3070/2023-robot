package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PathPlanningUtils;

public class AutoScoringTrajectoryCommand extends DriveToPointCommand {

    private static final Translation2d WALL_SIDE_MIDPOINT = new Translation2d(2.25, 0.9);
    private static final Translation2d LOADING_ZONE_MIDPOINT = new Translation2d(2.32, 4.53); // TODO the numbers are incorrect for LZ midpoint

    private final int scoringNode;

    /**
     * Constructs an AutoScoringTrajectoryCommand that will create and follow a trajectory 
     * bringing the robot to the given scoring node
     * 
     * @param scoringNode target node
     * @param constraints PathConstraints for the trajectory (maximum velocity and maximum acceleration)
     * @param autoBuilder the autobuilder that will be used to follow the trajectory
     * @param swerve the swerve subsystem
     */
    public AutoScoringTrajectoryCommand(int scoringNode, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        super(List.of(), constraints, autoBuilder, swerve);
        this.scoringNode = scoringNode;
    }

    @Override
    public void initialize() {
        Translation2d midPoint = null;
        Alliance alliance = DriverStation.getAlliance();

        Translation2d startTranslation = swerve.getPose().getTranslation(); // There might be a slight lag between initialize functions, could set current pose as a variable in super
        double x = startTranslation.getX();
        double y = startTranslation.getY();

        // TODO handle case where robot would have to back track to reach midpoint but would crash into charge station if it took a normal path

        if (x > 2.416 && x < 4.909) {
            if (alliance == DriverStation.Alliance.Blue) {
                if ( y > 0 && y < 1.509) {
                    midPoint = WALL_SIDE_MIDPOINT;
                } else if (y > 4.035 && y < 5.353) {
                    midPoint = LOADING_ZONE_MIDPOINT;
                }
            } else { // Red
                if (y > Constants.fieldWidthMeters && y < Constants.fieldWidthMeters - 1.509) {
                    midPoint = WALL_SIDE_MIDPOINT;
                } else if (y > Constants.fieldWidthMeters - 4.035 && y < Constants.fieldWidthMeters - 5.353) {
                    midPoint = LOADING_ZONE_MIDPOINT;
                }
            }
        }
        
        if (midPoint != null) {
            midPoint = PathPlanningUtils.transformTranslationForAlliance(midPoint, alliance);
        }

        Translation2d endTranslation = PathPlanningUtils.getRobotScoringPosition(scoringNode, alliance);
        
        Translation2d prevWaypoint = startTranslation;

        if (midPoint != null) {
            prevWaypoint = midPoint;
            
            // add mid point
            points.add(new PathPoint(midPoint, angleFromPosition(endTranslation, startTranslation))); // Might have to add control lengths
        }
        // add end point
        points.add(0, new PathPoint(endTranslation, angleFromPosition(endTranslation, prevWaypoint), new Rotation2d(Math.PI)).withPrevControlLength(endTranslation.getDistance(prevWaypoint) / 2));
        
        super.initialize();
    }

    /**
     * Calculates the angle (relative to the first position) of the hypoteneuse between two positions
     * 
     * @param position translation that the angle is relative to
     * @param other translation that determines the angle
     * @return
     */
    private Rotation2d angleFromPosition(Translation2d position, Translation2d other) {
        Translation2d translationDifference = other.minus(position);
        return new Rotation2d(Math.atan2(translationDifference.getY(), translationDifference.getX()));
    }
}
