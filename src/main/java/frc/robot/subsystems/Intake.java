package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GameObject;

public class Intake extends SubsystemBase {
    
    private final WPI_TalonSRX talIntake;

    private final DigitalInput openedSwitch;

    private GameObject gameObject = GameObject.NONE;

    public Intake() {
        talIntake = new WPI_TalonSRX(Constants.Intake.ID);
        talIntake.configFactoryDefault();
        talIntake.setInverted(Constants.Intake.inverted);
        talIntake.setNeutralMode(NeutralMode.Brake);

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

    public GameObject getGameObject() {
        return gameObject;
    }

    public void setGameObject(GameObject object) {
        gameObject = object;
    }

    public Command closeCommand() {
        return run(() -> set(Constants.Intake.closeVelocity)).withTimeout(Constants.Intake.closeDuration);
    }

    public Command openCommand() {
        return run(() -> set(Constants.Intake.openVelocity)).until(() -> openedSwitch.get());
    }
}
