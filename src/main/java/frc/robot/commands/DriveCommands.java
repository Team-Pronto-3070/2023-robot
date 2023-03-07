package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SwerveModuleState2;

public class DriveCommands extends CommandBase{

    /**
     * Command for driving onto charge station and balancing using PID control
     * 
     * Assumes robot is in position to ascend charge station by driving in x direction only
     * and is in the desired orientation for its ascent
     * 
     * @param swerve the swerve subsystem
     */
    private static Command autoBalancePID(SwerveSubsystem swerve) {

        return sequence(
            // drive onto ramp
            parallel(
                swerve.run(() -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * ((swerve.getPose().getX() < 4) ? 1 : -1), 0, 0, true, false)),
                new WaitUntilCommand(() -> swerve.getPitch() >= Constants.DriveCommands.AutoBalance.onRampAngle)
            ),

            // balance
            new PIDCommand(
                new PIDController(Constants.DriveCommands.AutoBalance.PID.P, Constants.DriveCommands.AutoBalance.PID.I, Constants.DriveCommands.AutoBalance.PID.D), 
                swerve::getPitch,
                Constants.DriveCommands.AutoBalance.balanceSetpoint, 
                (output) -> swerve.drive(
                    MathUtil.clamp(-output, -Constants.DriveCommands.AutoBalance.driveUpRampSpeed, Constants.DriveCommands.AutoBalance.driveUpRampSpeed) * ((swerve.getPose().getX() < 4) ? 1 : -1), 
                    0, 
                    0, 
                    true, 
                    false), 
                swerve));
    }

    /**
     * Command for driving onto the charge station and balancing by driving up charge station at a static speed and stopping
     * 
     * Assumes robot is in position to ascend charge station by driving in x direction only
     * and is in the desired orientation for its ascent
     * 
     * @param swerve the swerve subsystem
     * @return
     */
    private static Command autoBalanceBangBang(SwerveSubsystem swerve) {
        SwerveModuleState2[] stopStates = {
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(45)), 0), // front left
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(-45)), 0), // front right
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(-45)), 0), // rear left
            new SwerveModuleState2(0, new Rotation2d(Units.degreesToRadians(45)), 0), // rear right
        };

        return sequence(
            parallel(
                swerve.run(
                    () -> swerve.drive(Constants.DriveCommands.AutoBalance.driveUpRampSpeed * ((swerve.getPose().getX() < 4) ? 1 : -1), 0, 0, true, false)),
                sequence(
                    new WaitUntilCommand(() -> swerve.getPitch() > Constants.DriveCommands.AutoBalance.onRampAngle),
                    new WaitUntilCommand(() -> swerve.getPitch() < Constants.DriveCommands.AutoBalance.stopAngle)
                )
            ),
            swerve.run(() -> swerve.setModuleStates(stopStates))
        );
    }

    public static Command autoBalance(SwerveSubsystem swerve) {
        // return autoBalanceBangBang(swerve);
        return autoBalancePID(swerve);
    }
}
