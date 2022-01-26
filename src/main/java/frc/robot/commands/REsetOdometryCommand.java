package frc.robot.commands;

import static frc.robot.Constants.LimeConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.DriveTrain;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class REsetOdometryCommand extends CommandBase {
    public boolean done = false;
    public LimeLight lime;
    public DriveTrain driveTrain;
    public REsetOdometryCommand(DriveTrain driveTrain, LimeLight limelight) {
        this.lime = limelight;
        this.driveTrain = driveTrain;
    }

    public void execute(){
        lime.update();
        Pose2d pos = new Pose2d(lime.getDistaceToTarget() * inchesToMeters, 0.0 , new Rotation2d(0.0));
        SmartDashboard.putNumber("Distance to Target", lime.getDistaceToTarget() * inchesToMeters);
        //System.out.println("Shut UP VLAD");
        driveTrain.resetOdometry(pos);
        done = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return done;
    }

    //@Override
    public void end(){
        System.out.println("The command has fdasljjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjkended");
    }
}
