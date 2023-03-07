package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameObject;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AntiTipper;

public class TeleopDriveCommand extends CommandBase {
    private final SwerveSubsystem swerve;
    private final OI oi;
    private final boolean isOpenLoop, fieldRelative, useAbsoluteAngle, useAntiTipper;

    private PIDController thetaController;
    private AntiTipper antiTipper;
    private double lastAngle;
    
    public TeleopDriveCommand(SwerveSubsystem swerve, OI oi, boolean isOpenLoop, boolean fieldRelative, boolean useAbsoluteAngle) {
        this(swerve, oi, isOpenLoop, fieldRelative, useAbsoluteAngle, false, null, null, null);
    }

    // note that the AntiTipper only works in field-relative mode
    public TeleopDriveCommand(SwerveSubsystem swerve, OI oi, boolean isOpenLoop, boolean fieldRelative, boolean useAbsoluteAngle, boolean useAntiTipper,
                              Supplier<Rotation2d> armAngleSupplier, DoubleSupplier elevatorPositionSupplier, Supplier<GameObject> intakeContents) {
        this.swerve = swerve;
        this.oi = oi;
        this.isOpenLoop = isOpenLoop;
        this.fieldRelative = fieldRelative;
        this.useAbsoluteAngle = useAbsoluteAngle;
        this.useAntiTipper = useAntiTipper;

        if (useAbsoluteAngle) {
            thetaController = new PIDController(1, 0, 0); //TODO
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            lastAngle = 0;
        }

        if (useAntiTipper) {
            antiTipper = new AntiTipper(armAngleSupplier, elevatorPositionSupplier, intakeContents);
        }

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x, y, omega;
        
        if (useAbsoluteAngle) {
            double angle = Math.hypot(oi.absoluteHeadingHorizontal.getAsDouble(), oi.absoluteHeadingVertical.getAsDouble()) < 0.5 ?
                lastAngle : Math.atan2(oi.absoluteHeadingHorizontal.getAsDouble(), oi.absoluteHeadingVertical.getAsDouble());
            omega = thetaController.calculate(swerve.getYaw().getRadians(), angle) * Constants.Swerve.maxAngularSpeed;
            lastAngle = angle;
        } else {
            omega = oi.processed_drive_rot.getAsDouble();
        }

        if (useAntiTipper && fieldRelative) {
            ChassisSpeeds currentSpeeds = swerve.getChassisSpeeds();
            Translation2d velocity = antiTipper.limitVelocity(
                swerve.getYaw(),
                new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
                new Translation2d(oi.processed_drive_x.getAsDouble(), oi.processed_drive_y.getAsDouble())
            );
            x = velocity.getX();
            y = velocity.getY();
        } else {
            x = oi.processed_drive_x.getAsDouble();
            y = oi.processed_drive_y.getAsDouble();
        }

        swerve.drive(x, y, omega, fieldRelative, isOpenLoop);
    }
}
