package frc.robot;

import java.util.function.BooleanSupplier;
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

    public final BooleanSupplier driveSlow;

    public final Trigger armToNextTargetPositionButton;
    public final Trigger armToShelfIntakePositionButton;
    public final Trigger armToGroundIntakePositionButton;
    public final Trigger armToHomePositionButton;

    public final Trigger closeIntakeButton;
    public final Trigger openIntakeButton;

    public final Trigger driveToScoringNodeButton;
    public final Trigger gyroResetButton;
    public final Trigger interruptButton;

    public final Trigger targetLvl1ArmPosition;
    public final Trigger targetLvl2ArmPosition;
    public final Trigger targetLvl3ArmPosition;

    public final Trigger manualArmButton;
    public final Trigger manualArmTrigger;
    public final DoubleSupplier manualArmVerticalPower;
    public final DoubleSupplier manualArmElevatorPower;

    public final Trigger targetSlot1;
    public final Trigger targetSlot2;
    public final Trigger targetSlot3;
    public final Trigger targetSlot4;
    public final Trigger targetSlot5;
    public final Trigger targetSlot6;
    public final Trigger targetSlot7;
    public final Trigger targetSlot8;
    public final Trigger targetSlot9;


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

        openIntakeButton = driver.leftTrigger();
        closeIntakeButton = driver.rightTrigger();

        armToNextTargetPositionButton = driver.y();
        armToShelfIntakePositionButton = driver.b();
        armToGroundIntakePositionButton = driver.a();
        armToHomePositionButton = driver.x();

        driveSlow = () -> driver.rightBumper().getAsBoolean();
        driveToScoringNodeButton = driver.leftBumper();
        gyroResetButton = driver.povRight();
        interruptButton = driver.start();

        operator = new CommandXboxController(operatorPort);

        targetLvl1ArmPosition = operator.povDown();
        targetLvl2ArmPosition = operator.povLeft();
        targetLvl3ArmPosition = operator.povUp();

        manualArmVerticalPower = () -> MathUtil.applyDeadband(-operator.getLeftY(), Constants.OI.deadband, Constants.OI.maxManualRotationSpeed);
        manualArmElevatorPower = () -> MathUtil.applyDeadband(-operator.getRightY(), Constants.OI.deadband, Constants.OI.maxManualExtensionSpeed);

        manualArmTrigger = new Trigger(() -> manualArmElevatorPower.getAsDouble() != 0 || manualArmVerticalPower.getAsDouble() != 0);
        manualArmButton = operator.rightTrigger(Constants.OI.triggerDeadband);

        BooleanSupplier leftGrid = () -> operator.leftBumper().getAsBoolean() && !operator.rightBumper().getAsBoolean();
        BooleanSupplier centerGrid = () -> operator.rightBumper().getAsBoolean() && operator.leftBumper().getAsBoolean();
        BooleanSupplier rightGrid = () -> operator.rightBumper().getAsBoolean() && !operator.leftBumper().getAsBoolean();

        targetSlot1 = operator.x().and(leftGrid);
        targetSlot2 = operator.a().and(leftGrid);
        targetSlot3 = operator.b().and(leftGrid);
        targetSlot4 = operator.x().and(centerGrid);
        targetSlot5 = operator.a().and(centerGrid);
        targetSlot6 = operator.b().and(centerGrid);
        targetSlot7 = operator.x().and(rightGrid);
        targetSlot8 = operator.a().and(rightGrid);
        targetSlot9 = operator.b().and(rightGrid);
    }
}