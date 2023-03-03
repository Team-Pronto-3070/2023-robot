package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
    private CommandXboxController driver;
    private CommandXboxController operator;

    public final DoubleSupplier drive_x;
    public final DoubleSupplier drive_y;
    public final DoubleSupplier drive_rot;

    public final DoubleSupplier processed_drive_x;
    public final DoubleSupplier processed_drive_y;
    public final DoubleSupplier processed_drive_rot;
    
    public final DoubleSupplier absoluteHeadingHorizontal;
    public final DoubleSupplier absoluteHeadingVertical;

    public final Trigger gyroResetButton;

    public final DoubleSupplier manualArmVerticalPower;
    public final DoubleSupplier manualArmElevatorPower;

    public final Trigger manualArmButton;


    public OI(int driverPort, int operatorPort) {
        driver = new CommandXboxController(driverPort);

        drive_x = () -> -driver.getLeftY();
        drive_y = () -> -driver.getLeftX();
        drive_rot = () -> -driver.getRightX();

        processed_drive_x = () -> Math.pow(MathUtil.applyDeadband(drive_x.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed), 3);
        processed_drive_y = () -> Math.pow(MathUtil.applyDeadband(drive_y.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed), 3);
        processed_drive_rot = () -> MathUtil.applyDeadband(drive_rot.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxAngularSpeed);

        absoluteHeadingHorizontal = () -> -driver.getRightX();
        absoluteHeadingVertical = () -> -driver.getRightY();

        gyroResetButton = driver.x();

        operator = new CommandXboxController(operatorPort);

        manualArmVerticalPower = () -> MathUtil.applyDeadband(-operator.getLeftY(), Constants.OI.deadband, Constants.ElevatorArm.maxManualRotationSpeed);
        manualArmElevatorPower = () -> MathUtil.applyDeadband(-operator.getRightY(), Constants.OI.deadband, Constants.ElevatorArm.maxManualExtensionSpeed);

        manualArmButton = operator.rightTrigger(Constants.OI.triggerDeadband);
    }
}
