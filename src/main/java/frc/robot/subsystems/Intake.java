package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static final double closeVelocity = 1;
    private static final double openVelocity = -1;
    
    private final WPI_TalonSRX talIntake;

    private final DigitalInput closedSwitch;
    private final DigitalInput openedSwitch;

    public Intake() {
        talIntake = new WPI_TalonSRX(Constants.Intake.ID);
        talIntake.configFactoryDefault();
        talIntake.setInverted(false); //TODO which way is the motor?
        talIntake.setNeutralMode(NeutralMode.Brake);

        closedSwitch = new DigitalInput(Constants.Intake.closedSwitchPort);
        openedSwitch = new DigitalInput(Constants.Intake.openedSwitchPort);
    }

    /**
     * Sets the intake to the given velocity
     * 
     * @param velocity desired speed of the intake
     */
    public void set(double velocity) {
        talIntake.set(velocity);
    }

    /**
     * Closes the intake
     */
    public void close() {
        set(closeVelocity);
    }

    /**
     * Opens the intake
     */
    public void open() {
        set(openVelocity);
    }

    /**
     * Gets the value of the switch that determines if the intake is fully opened
     */
    public boolean getOpenedSwitch() {
        return openedSwitch.get();
    }

    /**
     * Gets the value of the switch that determines if the intake is fully closed
     */
    public boolean getClosedSwitch() {
        return closedSwitch.get();
    }
}
