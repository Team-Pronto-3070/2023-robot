package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private Translation2d pivotOffset = new Translation2d(Constants.MassProperties.pivotLocation.getX(),
                                                          Constants.MassProperties.pivotLocation.getZ());

    public ElevatorArmSubsystem() {
        verticalTalon = new WPI_TalonSRX(Constants.ElevatorArm.VerticalDrive.ID);
        verticalTalon.configFactoryDefault();
        verticalTalon.configAllSettings(Constants.ElevatorArm.VerticalDrive.config);
        verticalTalon.setNeutralMode(NeutralMode.Brake);
        verticalTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        SmartDashboard.putNumber("initial pivot absolute sensor units", verticalTalon.getSensorCollection().getPulseWidthPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 1", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 2", verticalTalon.getSelectedSensorPosition());
        //verticalTalon.setSelectedSensorPosition(Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset - verticalTalon.getSensorCollection().getPulseWidthPosition());
        verticalTalon.setSelectedSensorPosition(verticalTalon.getSensorCollection().getPulseWidthPosition() - Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset);
        SmartDashboard.putNumber("initial pivot relative sensor units 3", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 4", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 5", verticalTalon.getSelectedSensorPosition());
        verticalTalon.setSensorPhase(Constants.ElevatorArm.VerticalDrive.sensorPhase);
        verticalTalon.setInverted(Constants.ElevatorArm.VerticalDrive.motorReversed);
        SmartDashboard.putNumber("initial pivot relative sensor units 6", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 7", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 8", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 9", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 10", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 11", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units 12", verticalTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("initial pivot relative sensor units from sensor collection", verticalTalon.getSensorCollection().getQuadraturePosition());
       // verticalTalon.getSensorCollection().syncQuadratureWithPulseWidth(
       //                     3148,
       //                     2305,
       //                     false,
       //                     -((int) Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset),
       //                     0);
        
        elevatorTalon = new WPI_TalonSRX(Constants.ElevatorArm.ElevatorDrive.ID);
        elevatorTalon.configFactoryDefault();
        elevatorTalon.configAllSettings(Constants.ElevatorArm.ElevatorDrive.config);
        elevatorTalon.setSensorPhase(Constants.ElevatorArm.ElevatorDrive.sensorPhase);
        elevatorTalon.setInverted(Constants.ElevatorArm.ElevatorDrive.motorReversed);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);
        elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        resetTarget();
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
            rawFromExtention(
                Math.min( // keep inside of the robot bounds
                    target_extention,
                    calcMaxExtention(target_angle)
                )
            )
        );
        SmartDashboard.putNumber("motion magic extention setpoint",
            rawFromExtention(
                Math.min( // keep inside of the robot bounds
                    target_extention,
                    calcMaxExtention(target_angle)
                )
            )
        );
    }

    /**
     * @return the maximum extention of the arm at a certian 
     * angle to respect the extention bounds
     */
    private double calcMaxExtention(Rotation2d angle) {
        return 1000.0;
/*
        return Math.min(
            (Constants.RobotBounds.maxHorizontalExtention + (Constants.RobotBounds.robotLength / 2.0) + Constants.MassProperties.pivotLocation.getX())
            / angle.getCos(),

            (Constants.RobotBounds.maxHeight - Constants.MassProperties.pivotLocation.getZ())
            / angle.getSin()
        );
*/

//        return Math.min(
//            angle.getSin() // calc max horizontal extention
//            * (Constants.RobotBounds.maxHorizontalExtention + Constants.RobotBounds.robotLength - Constants.MassProperties.pivotLocation.getX()),
//            
//            angle.getCos() // calc max vertical extention
//            * (Constants.RobotBounds.maxHeight - Constants.MassProperties.pivotLocation.getZ())
//        );
    }

    private void setTarget(Translation2d target) {
        //target_angle = target.minus(pivotOffset).getAngle(); // first gets the relative target to the arm
        //target_extention = target.getDistance(pivotOffset); // get the offset
        target_angle = target.getAngle();
        target_extention = target.getNorm();
    }

    /**
     * 
     * @return Rotation2d - the angle of the arm
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                verticalTalon.getSelectedSensorPosition() // raw sensor units
//                - Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset) // the offset of the encoder in raw units
                / 4096.0 // revolutions
            );
    }

    /**
     * 
     * @return double - the extention of the elevator in meters
     */
    public double getExtention() {
        return (((elevatorTalon.getSelectedSensorPosition() // raw sensor units
            / 4096.0) // revolutions before gear ratio
            / Constants.ElevatorArm.ElevatorDrive.gearRatio) // final revolutions
            * Constants.ElevatorArm.ElevatorDrive.pulleyCircumference) // extention distance in meters
            + Constants.ElevatorArm.initialArmLength; // full arm length
    }

    /**
     * @param sAngle
     * @return raw sensor units for TalonSRX of vertical
     */
    private double rawFromAngle(Rotation2d sAngle) {
        return sAngle.getRotations() // total revolutions
                * 4096.0; // raw sensor units
//                + Constants.ElevatorArm.VerticalDrive.absoluteEncoderOffset;
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

    /**
     * sets both of the motors to 0
     */
    public void stop() {
        verticalTalon.set(0);
        elevatorTalon.set(0);
    }

    /**
     * 
     * @return command that moves to the target then stops
     */
    public Command goToTargetCommand(Constants.ElevatorArm.Position target) {
        return this.runOnce(() -> this.setTarget(target.translation))
                   .andThen(
                       this.run(this::move)
                       .until(this::atTarget)
                   );
    }

    /**
     * @return command to power the motors directlty
     */
    public Command manualMoveCommand(DoubleSupplier verticalPower, DoubleSupplier elevatorPower) {

        return this.run(() -> {
            verticalTalon.set(verticalPower.getAsDouble());
            elevatorTalon.set(elevatorPower.getAsDouble());

            resetTarget();
        });
    }

    /**
     * resets target_angle & target_extention to current angle & extention
     */
    public void resetTarget() {
        target_angle = getAngle();
        target_extention = getExtention();
    }

    public boolean atTarget() {
        return 
            (Math.abs(getAngle().minus(target_angle).getRadians()) < Constants.ElevatorArm.VerticalDrive.tolerance.getRadians()) &&
            (Math.abs(getExtention() - target_extention) < Constants.ElevatorArm.ElevatorDrive.tolerance);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("extention distance meters", getExtention());
        SmartDashboard.putNumber("extention distance sensor units", elevatorTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("pivot distance degrees", getAngle().getDegrees());
        SmartDashboard.putNumber("pivot distance sensor units", verticalTalon.getSelectedSensorPosition());

        SmartDashboard.putNumber("pivot absolute position sensor units", verticalTalon.getSensorCollection().getPulseWidthPosition());

        SmartDashboard.putBoolean("lower limit switch", elevatorTalon.isRevLimitSwitchClosed() == 1);
        SmartDashboard.putBoolean("upper limit switch", elevatorTalon.isFwdLimitSwitchClosed() == 1);

        SmartDashboard.putNumber("target_extention", target_extention);
        SmartDashboard.putNumber("target_angle", target_angle.getDegrees());
        SmartDashboard.putNumber("max extention", calcMaxExtention(getAngle()));
    }
}
