// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final OI oi = new OI(Constants.OI.driverPort, Constants.OI.operatorPort);

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorArmSubsystem elevatorArm = new ElevatorArmSubsystem();

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
            MathUtil.applyDeadband(oi.drive_x.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed),
            MathUtil.applyDeadband(oi.drive_y.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed),
            MathUtil.applyDeadband(oi.drive_rot.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxAngularSpeed),
            true,
            true
        )));
    elevatorArm.setDefaultCommand(elevatorArm.run(elevatorArm::move));

    configureBindings();
  }

  private void configureBindings() {
    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::resetGryo));
    
    Trigger manualArmTrigger = new Trigger(() -> oi.manualArmElevatorPower.getAsDouble() != 0 || oi.manualArmVerticalPower.getAsDouble() != 0);
    oi.manualArmButton.and(manualArmTrigger).whileTrue(elevatorArm.manualMoveCommand(oi.manualArmVerticalPower, oi.manualArmElevatorPower));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
