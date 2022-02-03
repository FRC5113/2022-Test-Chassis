/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CenterTargetRobot;
import frc.robot.commands.TestCommand;
import frc.robot.commands.UpdateLimeCommand;
import frc.robot.commands.SequentialTurretCommand;
import frc.robot.commands.REsetOdometryCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import static frc.robot.Constants.ChassisConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final DriveTrain driveTrain = new DriveTrain();
  public final LimeLight lime = new LimeLight();
  public Joystick controllerLeft = new Joystick(2);
  public Joystick controllerRight = new Joystick(0);
  public JoystickButton leftTrigger = new JoystickButton(controllerLeft, 1);
  public JoystickButton rightTrigger = new JoystickButton(controllerRight, 1);
  public JoystickButton rightButton = new JoystickButton(controllerRight, 4);
  public JoystickButton leftButtonThree = new JoystickButton(controllerLeft, 3);
  public JoystickButton rightButtonThree = new JoystickButton(controllerRight, 3);
  public JoystickButton rightButtonTwo = new JoystickButton(controllerLeft, 2);
  public XboxController driveController = new XboxController(3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public double getLeft() {
    // System.out.print("Left: ");
    // System.out.println(controllerLeft.getRawAxis(1));
    return controllerLeft.getRawAxis(1);

  }

  public boolean getLeftTrigger() {
    return controllerRight.getTrigger();
  }

  public double getRight() {
    // System.out.print("Right: ");
    // System.out.println(controllerRight.getRawAxis(1));
    return controllerRight.getRawAxis(1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driveController, 1).whileHeld(new
    // CenterTargetRobot(driveTrain, lime));
    // rightTrigger.toggleWhenActive(new CenterTargetRobot(driveTrain, lime));
    leftTrigger.and(rightTrigger)
        .toggleWhenActive(new CenterTargetRobot(driveTrain, lime).andThen(new REsetOdometryCommand(driveTrain, lime)));
    // rightTrigger.whileHeld(new TestCommand(driveTrain));
    leftButtonThree.toggleWhenActive(new UpdateLimeCommand(lime));
    // rightButtonThree.toggleWhenActive(new SequentialTurretCommand(driveTrain,
    // lime));
    rightButtonThree.toggleWhenActive(new REsetOdometryCommand(driveTrain, lime));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    class DriveConstants {
      // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
      // These characterization values MUST be determined either experimentally or
      // theoretically
      // for *your* robot's drive.
      // The Robot Characterization Toolsuite provides a convenient tool for obtaining
      // these
      // values for your robot.
      public static final double ksVolts = 0.50207;
      public static final double kvVoltSecondsPerMeter = 0.0097242;
      public static final double kaVoltSecondsSquaredPerMeter = 0.00038467;

      // Example value only - as above, this must be tuned for your drive!
      public static final double kPDriveVel = 0.014779;
      public static final double kTrackwidthMeters = 0.40132;
      // public static final DifferentialDriveKinematics kDriveKinematics = ;
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      // Reasonable baseline values for a RAMSETE follower in units of meters and
      // seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
    }
    DifferentialDriveKinematics difDrive = new DifferentialDriveKinematics(
      DriveConstants.kTrackwidthMeters);

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        difDrive,
        10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(difDrive)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveTrain::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        difDrive,
        driveTrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveTrain::tankDriveVolts,
        driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
