package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoScoringTrajectoryCommand extends DriveToPointCommand {

    public AutoScoringTrajectoryCommand(PathPoint endPoint, PathConstraints constraints, SwerveAutoBuilder autoBuilder, SwerveSubsystem swerve) {
        super(endPoint, constraints, autoBuilder, swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();

        // TODO: doesnt work for first 2 nodes

        if (x > 2.416 && x < 4.909 && y > 0 && y < 1.509) {
            // wall side of blue community
            points.add(0, new PathPoint(new Translation2d(2.32, 0.084), new Rotation2d(-0.593412)));
        }
        if (x > 2.416 && x < 4.909 && y > 4.035 && y < 5.353) {

            // loading zone side of blue community
            points.add(0, new PathPoint(new Translation2d(2.32, 4.53), new Rotation2d(0.558505)));
        }

        super.initialize();
    }
}
