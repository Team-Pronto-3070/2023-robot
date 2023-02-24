package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    public final SwerveAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem swerve) {

        // TODO - map to commands
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("armToGroundIntake", new InstantCommand());
        eventMap.put("armToShelfIntake", new InstantCommand());
        eventMap.put("armToLevel1Cone", new InstantCommand());
        eventMap.put("armToLevel1Cube", new InstantCommand());
        eventMap.put("armToLevel2Cone", new InstantCommand());
        eventMap.put("armToLevel2Cube", new InstantCommand());
        eventMap.put("armToLevel3Cone", new InstantCommand());
        eventMap.put("armToLevel3Cube", new InstantCommand());
        eventMap.put("resetArm", new InstantCommand());
        eventMap.put("scoreLevel1Cone", new InstantCommand());
        eventMap.put("scoreLevel1Cube", new InstantCommand());
        eventMap.put("scoreLevel2Cone", new InstantCommand());
        eventMap.put("scoreLevel2Cube", new InstantCommand());
        eventMap.put("scoreLevel3Cone", new InstantCommand());
        eventMap.put("scoreLevel3Cube", new InstantCommand());
        eventMap.put("intakeCone", new InstantCommand());
        eventMap.put("intakeCube", new InstantCommand());
        eventMap.put("autoBalance", new InstantCommand());

        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, // Pose2d supplier
            swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            swerve.kinematics, // SwerveDriveKinematics
            new PIDConstants(Constants.Auto.TranslationPID.P, Constants.Auto.TranslationPID.I, Constants.Auto.TranslationPID.D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("none", new PrintCommand("no auto option selected"));
        autoChooser.addOption("0 Piece Taxi Balance", buildAuto("0 Piece Taxi Balance"));
        autoChooser.addOption("1 Piece Taxi LZ", buildAuto("1 Piece Taxi LZ"));
        autoChooser.addOption("2 Piece Balance LZ", buildAuto("2 Piece Balance LZ"));
        autoChooser.addOption("2 Piece No Balance LZ", buildAuto("2 Piece No Balance LZ"));
        autoChooser.addOption("3 Piece No Balance LZ", buildAuto("3 Piece No Balance LZ"));
        autoChooser.addOption("1 Piece Taxi WALL", buildAuto("1 Piece Taxi WALL"));
        autoChooser.addOption("2 Piece Balance WALL", buildAuto("2 Piece Balance WALL"));
        autoChooser.addOption("2 Piece No Balance WALL", buildAuto("2 Piece No Balance WALL"));
        autoChooser.addOption("3 Piece No Balance WALL", buildAuto("3 Piece No Balance WALL"));

    }

    public Command buildAuto(String name) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, new PathConstraints(Constants.Auto.maxVelocity, Constants.Auto.maxAcceleration)));
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
