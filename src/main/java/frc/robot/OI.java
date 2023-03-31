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

    public final Trigger armToShelfIntakePositionButton;
    public final Trigger armToGroundIntakePositionButton;
    public final Trigger armToHomePosition;

    public final Trigger scoreGamePiece;
    public final Trigger fullIntake;
    public final Trigger armToNextTargetPosition;
    public final Trigger driveSlow;

    public final Trigger goToCardinal;

    public final Trigger gyroResetButton;
    public final Trigger interruptButton;
    
    public final Trigger closeIntakeButton;
    public final Trigger openIntakeButton;

    public final Trigger targetLvl1ArmPosition;
    public final Trigger targetLvl2ArmPosition;
    public final Trigger targetLvl3ArmPosition;

    public final Trigger targetShelfIntake;
    public final Trigger targetFloorIntake;
    
    public final Trigger setGameObjectCone;
    public final Trigger setGameObjectCube;

    public final Trigger manualArmButton;
    public final DoubleSupplier manualArmVerticalPower;
    public final DoubleSupplier manualArmElevatorPower;

    public OI(int driverPort, int operatorPort) {
        driver = new CommandXboxController(driverPort);
        operator = new CommandXboxController(operatorPort);

        drive_x = () -> -driver.getLeftY();
        drive_y = () -> -driver.getLeftX();
        drive_rot = () -> -driver.getRightX();

        absoluteHeadingHorizontal = () -> -driver.getRightX();
        absoluteHeadingVertical = () -> -driver.getRightY();

        fullIntake = driver.rightBumper();
        armToNextTargetPosition = driver.leftBumper();
        scoreGamePiece = driver.leftTrigger(Constants.OI.triggerDeadband);
        driveSlow = driver.rightTrigger();
        
        goToCardinal = driver.y();

        //processed_drive_x = () -> Math.pow(MathUtil.applyDeadband(drive_x.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed), 3);
        //processed_drive_y = () -> Math.pow(MathUtil.applyDeadband(drive_y.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxSpeed), 3);
        //processed_drive_rot = () -> MathUtil.applyDeadband(drive_rot.getAsDouble(), Constants.OI.deadband, Constants.Swerve.maxAngularSpeed);
        processed_drive_x = () -> Math.pow(MathUtil.applyDeadband(drive_x.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxSpeed * (driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1);
        processed_drive_y = () -> Math.pow(MathUtil.applyDeadband(drive_y.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxSpeed * (driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1);
        processed_drive_rot = () -> Math.pow(MathUtil.applyDeadband(drive_rot.getAsDouble(), Constants.OI.deadband), 3) * Constants.Swerve.maxAngularSpeed * (driveSlow.getAsBoolean() ? 0.25 : 1);

        gyroResetButton = driver.povRight();
        interruptButton = driver.start().or(operator.start());

        armToShelfIntakePositionButton = driver.b();
        armToGroundIntakePositionButton = driver.a();
        armToHomePosition = driver.x().or(operator.povRight());

        targetLvl1ArmPosition = operator.povDown();
        targetLvl2ArmPosition = operator.povLeft();
        targetLvl3ArmPosition = operator.povUp();

        targetShelfIntake = operator.b();
        targetFloorIntake = operator.a();

        openIntakeButton = operator.leftBumper();
        closeIntakeButton = operator.rightBumper();

        manualArmVerticalPower = () -> MathUtil.applyDeadband(-operator.getLeftY(), Constants.OI.deadband) * Constants.OI.maxManualRotationSpeed;
        manualArmElevatorPower = () -> MathUtil.applyDeadband(-operator.getRightY(), Constants.OI.deadband) * Constants.OI.maxManualExtensionSpeed;

        manualArmButton = operator.rightTrigger(Constants.OI.triggerDeadband).and(() -> manualArmElevatorPower.getAsDouble() != 0 || manualArmVerticalPower.getAsDouble() != 0);

        setGameObjectCone = operator.y();
        setGameObjectCube = operator.x();
    }
}