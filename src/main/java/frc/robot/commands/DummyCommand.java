package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

// Currently removed: Limelight debugger lol
public class DummyCommand extends CommandBase {
    public DummyCommand() {}

	// Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // do nothing
        System.out.println("Dummy command start!");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Dummy command active!");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Dummy command end!");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
