package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
    private XboxController driver;

    public final DoubleSupplier drive_x;
    public final DoubleSupplier drive_y;
    public final DoubleSupplier drive_rot;

    public final Trigger gyroResetButton;

    public final Trigger elevatorArmHome;
    public final Trigger elevatorArmNextLevel;
    public final Trigger elevatorArmRetract;

    public OI(int driverPort) {
        driver = new XboxController(driverPort);

        drive_x = () -> -driver.getLeftY();
        drive_y = () -> -driver.getLeftX();
        drive_rot = () -> -driver.getRightX();

        gyroResetButton = new JoystickButton(driver, XboxController.Button.kX.value);

        elevatorArmHome = new JoystickButton(driver, XboxController.Button.kA.value);
        elevatorArmNextLevel = new JoystickButton(driver, XboxController.Button.kB.value);
        elevatorArmRetract = new JoystickButton(driver, XboxController.Button.kY.value);
    }
    
}
