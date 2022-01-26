/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

// Currently removed: Limelight debugger lol
public class UpdateLimeCommand extends CommandBase {
    /**
     * Creates a new DriveCommands.
     */
    private LimeLight lime;

    public UpdateLimeCommand(LimeLight lime) {
        // this.driveTrain = driveTrain;
        this.lime = lime;
        addRequirements(lime);
    }

	// Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //do nothing
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // driveTrain.setLeftSpeed();
        // System.out.println("hmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm");
        lime.update();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
