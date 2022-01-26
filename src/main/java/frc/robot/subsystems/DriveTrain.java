/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ChassisConstants.*;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  public WPI_TalonFX leftMaster = new WPI_TalonFX(kFLChassis);
  public WPI_TalonFX rightMaster = new WPI_TalonFX(kFRChassis);
  public WPI_TalonFX leftSlave = new WPI_TalonFX(kBLChassis);
  public WPI_TalonFX rightSlave = new WPI_TalonFX(kBRChassis);

  private MotorController leftSide = new MotorControllerGroup(leftMaster, leftSlave); //SpeedControllerGroup allows for multiple SpeedControllers to be linked together
  private MotorController rightSide = new MotorControllerGroup(rightMaster, rightSlave);

  private DifferentialDrive driveBase = new DifferentialDrive(leftMaster, rightMaster); //allows for us to 
  private AHRS gyro = new AHRS(SPI.Port.kMXP); //we might need to set the update rate to 60 hz

  // private DigitalInput beamBreakIn = new DigitalInput(9);
  // private DigitalOutput beamBreakOut = new DigitalOutput(8);

  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kSChassis, kVChassis, kAChassis);
  
  //private double kp = 0.001;
  //private double ki = 0.00001;
  //private double kd = 0.1;
  
  //private double setPoint = 100;
  
  //private PIDController pid = new edu.wpi.first.math.controller.PIDController(kp, ki, kd);

  public DriveTrain() {

    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    
    leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
    rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());  
    
    leftMaster.config_kP(0, 0.25, 30);
    leftMaster.config_kI(0, 0.00025, 30);
    leftMaster.config_kD(0, 250, 30);
    //leftMaster.config_kF(0, (1023.0*1/leftMaster.getSelectedSensorVelocity()));

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    leftSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rightSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    
    rightMaster.configClosedloopRamp(5);
    leftMaster.configClosedloopRamp(5);
    

    gyro.reset();

   // this.setMaxVoltage(0.5);

    
    gyro.enableLogging(true);
   // System.out.println(gyro.getFirmwareVersion());
    //System.out.println(gyro.isRotating());

    //pid.setSetpoint(setPoint);
    //leftMaster.set(ControlMode.Velocity, 2000);

  }

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    rightSlave.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
    leftSlave.setSelectedSensorPosition(0);
  }

  public Rotation2d getHeading() {
    //System.out.println("jerjejrejreje");
    return Rotation2d.fromDegrees(-Math.IEEEremainder(gyro.getAngle(), 360));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose) {
    gyro.reset();
    odometry.resetPosition(pose, new Rotation2d(gyro.getAngle()));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() 
  {
    return (new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(4), 
      rightMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(4))); // change the wheel radius
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public double getAverageEncoderDistance() {
    return (rightMaster.getSelectedSensorPosition()+leftMaster.getSelectedSensorPosition())/2;
  }

  public void setMaxVoltage(double max) {
    driveBase.setMaxOutput(12);
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public void driveCartesian(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, left);
    rightMaster.set(ControlMode.PercentOutput, -right);

    //SmartDashboard.putNumber("left", left);
    //SmartDashboard.putNumber("right", right);
    SmartDashboard.putNumber("rightFront", rightMaster.getSupplyCurrent());
    SmartDashboard.putNumber("leftFront", leftMaster.getSupplyCurrent());
    SmartDashboard.putNumber("rightBack", rightSlave.getSupplyCurrent());
    SmartDashboard.putNumber("leftBack", leftSlave.getSupplyCurrent());

    // SmartDashboard.putBoolean("beamBreakVal", beamBreakOut.get());

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(-rightVolts);
    driveBase.feed();
  }

  public void setLeftSpeed() 
  {
    leftMaster.set(ControlMode.Velocity, 10000);
    SmartDashboard.putNumber("Velocity:", leftMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Current:", leftMaster.getStatorCurrent());
    //System.out.println(leftMaster.getSupplyCurrent());
    
  }

  
  public double getLeftRPM() 
  {
    return leftMaster.getSelectedSensorVelocity();
  }
  public double getRightRPM() 
  {
    return rightMaster.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    //System.out.println(gyro.isConnected());
    //System.out.println(gyro.isCalibrating());
    //System.out.println("from here");
    //System.out.println(this.getHeading());
    odometry.update(getHeading(), leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition());
  }

}
