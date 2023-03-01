package frc.robot.commands;

import javax.tools.Diagnostic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommands extends CommandBase{

    /**
     * Command for driving onto charge station and balancing
     * 
     * Assumes robot is in position to ascend charge station by driving in x direction only
     * and is in the desired orientation for its ascent
     * 
     * @param swerve the swerve subsystem
     */
    public static SequentialCommandGroup autoBalance(SwerveSubsystem swerve) {

        return new SequentialCommandGroup(
            // drive onto ramp
            new RunCommand(() -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * ((swerve.getPose().getX() < 2.75) ? 1 : -1), 0, 0, true, false), swerve)
            .until(() -> swerve.getPitch() >= Constants.DriveCommands.AutoBalance.onRampAngle),

            // balance
            new PIDCommand(
                new PIDController(
                    Constants.DriveCommands.AutoBalance.PID.P, 
                    Constants.DriveCommands.AutoBalance.PID.I, 
                    Constants.DriveCommands.AutoBalance.PID.D), 
                swerve::getPitch, 
                Constants.DriveCommands.AutoBalance.balanceSetpoint, 
                (output) -> swerve.drive(MathUtil.clamp(-output, -Constants.DriveCommands.AutoBalance.driveUpRampSpeed, Constants.DriveCommands.AutoBalance.driveUpRampSpeed), 0, 0, true, false), 
                swerve));
    }

    public static SequentialCommandGroup autoBalanceDeadReckoning(SwerveSubsystem swerve) {
        int direction = ((swerve.getPose().getX() < 2.75) ? 1 : -1);

        //TODO check below
        SwerveModuleState[] stopStates = {
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))), // front left
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))), // front right
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))), // rear left
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))), // rear right
        };
        return new SequentialCommandGroup(
            new RunCommand(() -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * direction, 0, 0, true, false))
            .until(() -> swerve.getPitch() >= Constants.DriveCommands.AutoBalance.onRampAngle),

            new RunCommand(() -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * direction, 0, 0, true, false))
            .until(() -> swerve.getPitch() >= Constants.DriveCommands.AutoBalance.stopAngle),

            new RunCommand(() -> swerve.setModuleStates(stopStates), swerve)
        );
    }
}
