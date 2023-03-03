package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class ElevatorArmSubsystem extends SubsystemBase {

    private final WPI_TalonSRX verticalTalon;
    private final WPI_TalonSRX elevatorTalon;

    private Rotation2d target_angle = new Rotation2d();
    private double target_extention = 0.0; //meters from pivot point to top of elevator

    public ElevatorArmSubsystem() {
        verticalTalon = new WPI_TalonSRX(Constants.ElevatorArm.VerticalDrive.ID);
        verticalTalon.configFactoryDefault();
        verticalTalon.configAllSettings(Constants.ElevatorArm.VerticalDrive.config);
        verticalTalon.setInverted(Constants.ElevatorArm.VerticalDrive.motorReversed);
        verticalTalon.setNeutralMode(NeutralMode.Brake);
        verticalTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        verticalTalon.setSelectedSensorPosition(verticalTalon.getSensorCollection().getPulseWidthPosition());
        
        elevatorTalon = new WPI_TalonSRX(Constants.ElevatorArm.ElevatorDrive.ID);
        elevatorTalon.configFactoryDefault();
        elevatorTalon.configAllSettings(Constants.ElevatorArm.ElevatorDrive.config);
        elevatorTalon.setInverted(Constants.ElevatorArm.ElevatorDrive.motorReversed);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);
        elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    /**
     * sets the output of the vertical and elevator motors
     * to the values calculated by verticalTProfile and 
     * elevatorTProfile
     * 
     * @param deltaT - delta time in seconds
     */
    public void move() {
        verticalTalon.set(
            ControlMode.MotionMagic, // * not sure if just putting MotionMagic here in will work
            rawFromAngle(target_angle) // motion magic should calculate the TProfile
        );
        elevatorTalon.set(
            ControlMode.MotionMagic, 
            Math.min( // keep inside of the robot bounds
                rawFromExtention(target_extention),
                calcMaxExtention(target_angle))
        );
    }

    /**
     * @return the maximum extention of the arm at a certian 
     * angle to respect the extention bounds
     */
    private double calcMaxExtention(Rotation2d angle) {
        return Math.min(
            angle.getSin() // calc max horizontal extention
            * (Constants.RobotBounds.maxHorizontalExtention + Constants.RobotBounds.robotLength - Constants.MassProperties.pivotLocation.getX()),
            
            angle.getCos() // calc max vertical extention
            * (Constants.RobotBounds.maxHeight - Constants.MassProperties.pivotLocation.getZ())
        );
    }

    private void setTarget(Translation2d target) {
        target_angle = (target.minus(Constants.ElevatorArm.armOffset)).getAngle(); // first gets the relative target to the arm
        target_extention = target.getDistance(Constants.ElevatorArm.armOffset); // get the offset
    }

    /**
     * 
     * @return Rotation2d - the angle of the arm
     */
    public Rotation2d getAngle() {
        return new Rotation2d(
            Units.rotationsToRadians( // convert rotations to radians
                verticalTalon.getSelectedSensorPosition() // raw sensor units
                - Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset // the offset of the encoder in raw units
                / 4096.0 // revolutions
        ));
    }

    /**
     * 
     * @return double - the extention of the elevator in meters
     */
    public double getExtention() {
        return elevatorTalon.getSelectedSensorPosition() // raw sensor units
            / 4096.0 // revolutions before gear ratio
            / Constants.ElevatorArm.ElevatorDrive.gearRatio // final revolutions
            * Constants.ElevatorArm.ElevatorDrive.pulleyCircumference // extention distance in meters
            + Constants.ElevatorArm.initialArmLength; // full arm length
    }

    /**
     * @param sAngle
     * @return raw sensor units for TalonSRX of vertical
     */
    private double rawFromAngle(Rotation2d sAngle) {
        return sAngle.getRotations() // total revolutions
            * 4096.0; // raw sensor units
    }

    private double rawFromTargetAngle() {
        return rawFromAngle(target_angle)
            + Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset; // add the offset
    }

    /**
     * @param sExtention
     * @return raw sensor units for TalonSRX of extention
     */
    private double rawFromExtention(double sExtention) {
        return (sExtention // full arm length
            - Constants.ElevatorArm.initialArmLength) // extention distance in meters
            / Constants.ElevatorArm.ElevatorDrive.pulleyCircumference// final revolutions
            * Constants.ElevatorArm.ElevatorDrive.gearRatio // revolutions before gear ratio
            * 4096.0; // raw sensor units
    }

    /**
     * 
     * @return velocity of the vertical drive motor in radians/s
     */
    private double getVerticalDriveVel() {
        return Units.rotationsToRadians( // radians per second
            verticalTalon.getSelectedSensorVelocity() // raw talon units per 100ms
            * 10.0 // raw units per second
            / 4096.0 // motor revolutions per second
        );
    }

    /**
     * 
     * @return velocity of the vertical drive motor in radians/s
     */
    private double getElevatorDriveVel() {
        return Units.rotationsToRadians( // radians per second
            elevatorTalon.getSelectedSensorVelocity() // raw talon units per 100ms
            * 10.0 // raw units per second
            / 4096.0 // motor revolutions per second
        );
    }

    /**
     * sets the target to retract the elevator all the way
     */
    public void targetRetract() {
        target_extention = Constants.ElevatorArm.initialArmLength; // set the extention to fully retracted
    }

    public Command setTargetCommand(Constants.ElevatorArm.Position target) {
        return this.runOnce(() -> setTarget(target.translation));
    }

    public Command manualMoveCommand(DoubleSupplier verticalPower, DoubleSupplier elevatorPower) {

        return this.run(() -> {
            verticalTalon.set(verticalPower.getAsDouble());
            elevatorTalon.set(elevatorPower.getAsDouble());

            target_angle = getAngle();
            target_extention = getExtention();
        });
    }

}
