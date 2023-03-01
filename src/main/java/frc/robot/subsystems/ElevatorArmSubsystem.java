package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class ElevatorArmSubsystem extends SubsystemBase {

    private final WPI_TalonSRX verticalTalon;
    private final WPI_TalonSRX elevatorTalon;

    private Rotation2d target_angle = new Rotation2d();
    private double target_extention = 0.0;
    private TrapezoidProfile verticalTProfile;
    private TrapezoidProfile elevatorTProfile;

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
     * 
     * @return the TrapezoidProfile for the vertical motor
     */
    private TrapezoidProfile calcVerticalTrapezoidProfile(Rotation2d target_angle) {
        double targetPos = target_angle.getRadians();
    
        //TODO fill in the constants
        return new TrapezoidProfile(
            Constants.ElevatorArm.VerticalDrive.trapConstraints,
            new TrapezoidProfile.State(targetPos, 0.0),
            new TrapezoidProfile.State(getAngle().getRadians(), getVerticalDriveVel())
        );
    }

    /**
     * 
     * @return the TrapezoidProfile for the elevator motor
     */
    private TrapezoidProfile calcElevatorTrapezoidProfile(double target_extention) {

        //TODO fill in the constants
        return new TrapezoidProfile(
            Constants.ElevatorArm.ElevatorDrive.trapConstraints,
            new TrapezoidProfile.State(target_extention, 0.0),
            new TrapezoidProfile.State(getExtention(), getElevatorDriveVel())
        );
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
            ControlMode.MotionMagic,
            rawFromAngle(new Rotation2d(verticalTProfile.calculate(Constants.loopTime).position))
        );
        elevatorTalon.set(
            ControlMode.MotionMagic,
            rawFromExtention(elevatorTProfile.calculate(Constants.loopTime).position)
        );
    }

    private void setTarget(Translation2d target) {
        verticalTProfile = calcVerticalTrapezoidProfile(target.getAngle()); // calculate the vertical drive TrapezoidProfile
        elevatorTProfile = calcElevatorTrapezoidProfile(target.getDistance(Constants.ElevatorArm.armOffset)); // calculate the elevator drive TrapezoidProfile
    }

    /**
     * 
     * @return Rotation2d - the angle of the arm
     */
    public Rotation2d getAngle() {
        return new Rotation2d(
            Units.rotationsToRadians( // convert rotations to radians
                verticalTalon.getSelectedSensorPosition() // raw sensor units
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

    /**
     * @param sExtention
     * @return raw sensor units for TalonSRX of extention
     */
    private double rawFromExtention(double sExtention) {
        return sExtention // full arm length
            - Constants.ElevatorArm.initialArmLength // extention distance in meters
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
            verticalTalon.getSelectedSensorVelocity() // raw talon units
            * (10.0 / 4096.0) // motor revolutions per second
        );
    }

    /**
     * 
     * @return velocity of the vertical drive motor in radians/s
     */
    private double getElevatorDriveVel() {
        return Units.rotationsToRadians( // radians per second
            elevatorTalon.getSelectedSensorVelocity() // raw talon units
            * (10.0 / 4096.0) // motor revolutions per second
        );
    }

    /**
     * sets the target to retract the elevator all the way
     */
    public void targetRetract() {
        target_extention = Constants.ElevatorArm.initialArmLength; // set the extention to fully retracted
        verticalTProfile = calcVerticalTrapezoidProfile(target_angle); // calculate the vertical drive TrapezoidProfile
        elevatorTProfile = calcElevatorTrapezoidProfile(target_extention); // calculate the elevator drive TrapezoidProfile
    }

    public Command setTargetCommand(Constants.ElevatorArm.Position target) {
        return this.runOnce(() -> setTarget(target.translation));
    }

    public Command moveCommand(double verticalPower, double elevatorPower) {
        return this.runOnce(() -> {
            verticalTalon.set(verticalPower);
            elevatorTalon.set(elevatorPower);
        });
    }

}
