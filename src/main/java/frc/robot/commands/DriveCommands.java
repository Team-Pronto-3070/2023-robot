package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveCommands extends CommandBase{

    public static SequentialCommandGroup autoBalance(Swerve drive) {
         // TODO drive up diagonally?,
        // don't assume an orientation of robot, 
        // use vision to drive to a position on field/ drive specified distance
        // look into coordinate systems
        
        return new SequentialCommandGroup(
            // Drive until fully on ramp
            new RunCommand(() -> drive.drive(Constants.AutoBalance.driveUpRampSpeed, 0, 0, false, false))
                .until(() -> drive.getPitch() > Constants.AutoBalance.onRampPitch),
            // Use pid to balance on charge station
            new PIDCommand(
                new PIDController(
                    Constants.AutoBalance.PID.P, 
                    Constants.AutoBalance.PID.I, 
                    Constants.AutoBalance.PID.D),
                drive::getPitch, 
                Constants.AutoBalance.balanceSetpoint, 
                output -> drive.drive(MathUtil.clamp(-output, -1, 1), 0, 0, false, false), 
                drive));
    }
    
}
