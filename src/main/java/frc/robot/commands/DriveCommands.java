package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommands extends CommandBase{

    /*
     * Command for driving onto charge station and balancing
     */
    public static SequentialCommandGroup autoBalance(SwerveSubsystem drive) {
        // drive up diagonally?
        // don't assume an orientation of robot

        return new SequentialCommandGroup(
            // TODO get robot to correct orientation and position

            balanceOnChargeStation(drive));
    }

    /*
     * Command for balancing on charge station
     * 
     * Assumes robot is already on charge station
     */
    private static Command balanceOnChargeStation(SwerveSubsystem drive) {
        return new PIDCommand(
            new PIDController(
                Constants.DriveCommands.AutoBalance.PID.P, 
                Constants.DriveCommands.AutoBalance.PID.I, 
                Constants.DriveCommands.AutoBalance.PID.D), 
            drive::getPitch, 
            Constants.DriveCommands.AutoBalance.balanceSetpoint, 
            (output) -> drive.drive(MathUtil.clamp(output, -1, 1), 0, 0, false, false), 
            drive);
    }
    
}
