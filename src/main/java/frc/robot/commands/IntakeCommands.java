package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands extends CommandBase {

    /**
     * Command to open the intake
     * 
     * @param intake the intake subsystem
     */
    public static ParallelRaceGroup open(Intake intake) {
        return new RunCommand(intake::open, intake).until(intake::getOpenedSwitch);
    }

    /**
     * Command to close the intake
     * 
     * @param intake the intake subsystem
     */
    public static ParallelRaceGroup close(Intake intake) {
        return new RunCommand(intake::close, intake).until(intake::getClosedSwitch);
    }
}
