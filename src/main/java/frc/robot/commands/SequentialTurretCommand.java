package frc.robot.commands;

import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
public class SequentialTurretCommand extends SequentialCommandGroup {
    //public boolean done = false;
    public SequentialTurretCommand(DriveTrain drivetrain, LimeLight limelight) {
        super(new CenterTargetRobot(drivetrain, limelight), new REsetOdometryCommand(drivetrain, limelight));
        //done = true;
    }

    // Returns true when the command should end.
    /*@Override
    public boolean isFinished() 
    {
        return done;
    }*/
}
