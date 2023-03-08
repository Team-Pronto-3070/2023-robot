package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GameObject;
import frc.robot.util.PicoColorSensor;

public class IntakeSubsystem extends SubsystemBase {
    
    private final WPI_TalonSRX talIntake;

    private final DigitalInput leftSwitch;
    private final DigitalInput rightSwitch;

    private GameObject gameObject = GameObject.NONE;

    private final PicoColorSensor colorSensor;

    public IntakeSubsystem() {
        talIntake = new WPI_TalonSRX(Constants.Intake.ID);
        talIntake.configFactoryDefault();
        talIntake.setInverted(Constants.Intake.inverted);
        talIntake.setNeutralMode(NeutralMode.Brake);

        leftSwitch = new DigitalInput(Constants.Intake.leftSwitchPort);
        rightSwitch = new DigitalInput(Constants.Intake.rightSwitchPort);

        colorSensor = new PicoColorSensor();
    }

    /**
     * Sets the intake to the given velocity
     * 
     * @param velocity desired speed of the intake
     */
    public void set(double velocity) {
        talIntake.set(velocity);
    }

    public void stop() {
        talIntake.set(0);
    }

    public GameObject getGameObject() {
        return gameObject;
    }

    public void setGameObject(GameObject object) {
        gameObject = object;
    }

    public Command closeCommand() {
        return run(() -> set(Constants.Intake.closeVelocity))
               .withTimeout(Constants.Intake.closeDuration)
               .andThen(this::stop, this);
    }

    public Command openCommand() {
        return run(() -> set(Constants.Intake.openVelocity))
               .until(() -> leftSwitch.get() || rightSwitch.get())
               .withTimeout(Constants.Intake.openTimeout)
               .andThen(this::stop, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("left intake limit switch", leftSwitch.get());
        SmartDashboard.putBoolean("right intake limit switch", rightSwitch.get());

        SmartDashboard.putNumber("color sensor distance", colorSensor.getProximity0());
    }
}
