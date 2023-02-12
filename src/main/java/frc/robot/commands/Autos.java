package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SwerveSubsystem swerve;

    private final HashMap<String, Command> eventMap = new HashMap<>();
    public final SwerveAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem swerve) {
        this.swerve = swerve;

        eventMap.put("testprint", new PrintCommand("successfully ran a print command"));

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
        autoChooser.addOption("test1", buildAuto("test1"));
    }

    public Command buildAuto(String name) {
        return autoBuilder.fullAuto(PathPlanner.loadPath(name, new PathConstraints(Constants.Auto.maxVelocity, Constants.Auto.maxAcceleration)));
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
