package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.GameObject;
import frc.robot.util.PicoColorSensor;
import frc.robot.util.PicoColorSensor.RawColor;

public class IntakeSubsystem extends SubsystemBase {
    
    private final WPI_TalonSRX talIntake;

    private final DigitalInput leftSwitch;
    private final DigitalInput rightSwitch;
    private final PicoColorSensor colorSensor;
    private final Trigger hasObject;

    private GameObject gameObject = GameObject.NONE;

    public IntakeSubsystem() {
        talIntake = new WPI_TalonSRX(Constants.Intake.ID);
        talIntake.configFactoryDefault();
        talIntake.configAllSettings(Constants.Intake.config);
        talIntake.setInverted(Constants.Intake.inverted);
        talIntake.setNeutralMode(NeutralMode.Brake);

        leftSwitch = new DigitalInput(Constants.Intake.leftSwitchPort);
        rightSwitch = new DigitalInput(Constants.Intake.rightSwitchPort);

        colorSensor = new PicoColorSensor();
        hasObject = new Trigger(() -> colorSensor.getProximity0() > Constants.Intake.distanceThreshold)
                        .debounce(Constants.Intake.distanceDebounceTime);
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

    public void autoSetGameObject() {
        if (Constants.Intake.isCone.apply(colorSensor.getRawColor0())) {
            setGameObject(GameObject.CONE);
        } else if (Constants.Intake.isCube.apply(colorSensor.getRawColor0())) {
            setGameObject(GameObject.CUBE);
        } else {
            setGameObject(GameObject.NONE);
        }
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

    public Command autoCloseCommand() {
        return sequence(
            waitUntil(hasObject),
            new InstantCommand(this::autoSetGameObject),
            closeCommand()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("left intake limit switch", leftSwitch.get());
        SmartDashboard.putBoolean("right intake limit switch", rightSwitch.get());

        SmartDashboard.putNumber("color sensor distance", colorSensor.getProximity0());

        RawColor color = colorSensor.getRawColor0();
        SmartDashboard.putNumber("color sensor red", color.red);
        SmartDashboard.putNumber("color sensor green", color.green);
        SmartDashboard.putNumber("color sensor blue", color.blue);
        SmartDashboard.putNumber("color sensor ir", color.ir);

        switch (gameObject) {
            case CUBE:
                SmartDashboard.putString("intake contents", "cube");
                break;
            case CONE:
                SmartDashboard.putString("intake contents", "cone");
                break;
            default:
            case NONE:
                SmartDashboard.putString("intake contents", "none");
                break;
        }
    }
}
