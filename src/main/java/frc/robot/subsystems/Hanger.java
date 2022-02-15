package frc.robot.subsystems;


//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants.*;


public class Hanger extends SubsystemBase{
    // those constats are wrong  (KFLCHASSIS)
    public WPI_TalonFX frontClimber = new WPI_TalonFX(kFLChassis);
    public WPI_TalonFX backClimber = new WPI_TalonFX(kFLChassis);
    public WPI_TalonFX frontHook = new WPI_TalonFX(kFLChassis);
    public WPI_TalonFX backHook = new WPI_TalonFX(kFLChassis);
    // this is cosntants but im putting it here
    public double wheelDiamater = 4.0;
    public double ratio = 1.0;


    public Hanger() {
        frontClimber.configFactoryDefault();
        backClimber.configFactoryDefault();
        frontHook.configFactoryDefault();
        backHook.configFactoryDefault();


        frontClimber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        backClimber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        frontHook.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        backHook.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);


        frontClimber.setNeutralMode(NeutralMode.Brake);
        backClimber.setNeutralMode(NeutralMode.Brake);
        frontHook.setNeutralMode(NeutralMode.Brake);
        backHook.setNeutralMode(NeutralMode.Brake);

        resetEncoders();
        // might be a good idea to put other reset stuff here but no clude what is good and what is bad


    }
    public void resetEncoders() {
        frontClimber.setSelectedSensorPosition(0);
        backClimber.setSelectedSensorPosition(0);
        frontHook.setSelectedSensorPosition(0);
        backHook.setSelectedSensorPosition(0);
      }
    
      /*
    public double getHeight(WPI_TalonFX motor) {
        return motor.getSelectedSensorVelocity();
    }
    */
    public void frontClimberSetSpeed(double speed) {
        //double volts = distance *2 * Math.PI * wheelDiamater * ratio; // this math is probaly wrong but i dont care
        //motor.setVoltage(volts);
        frontClimber.set(speed);
    }
    public void backClimberSetSpeed(double speed) {
        //double volts = distance *2 * Math.PI * wheelDiamater * ratio; // this math is probaly wrong but i dont care
        //motor.setVoltage(volts);
        backClimber.set(speed);
    }
    public void frontHookSetSpeed(double speed){
        frontHook.set(speed);
    }
    public void backHookSetSpeed(double speed){
        backHook.set(speed);
    }




    public double getFrontClimberEncoderValue(){
        return frontClimber.getSelectedSensorPosition();
    }

    public double getBackClimberEncoderValue(){
        return backClimber.getSelectedSensorPosition();
    }

    public double getFrontHookEncoderValue(){
        return frontHook.getSelectedSensorPosition();
    }

    public double getbackHookEncoderValue(){
        return backHook.getSelectedSensorPosition();
    }
}

