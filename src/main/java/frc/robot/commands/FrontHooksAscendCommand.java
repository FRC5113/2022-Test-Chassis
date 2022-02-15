package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanger;

public class FrontHooksAscendCommand extends CommandBase { 
    private Hanger hanger;
    //private double distance;
    //private WPI_TalonFX motor;

    //Constants values
    private double GearboxRatio = 100;
    private double elevationSpeed = 5;
    private double pulleyDiameter = 4;
    private double maxFrontClimberEncoderValue = 60000; //arbitrary very big number

    public FrontHooksAscendCommand(Hanger hanger) {
        this.hanger = hanger;
        addRequirements(hanger);
    }
    @Override
    public void initialize() {
        hanger.frontClimberSetSpeed(0);
    }
    @Override
    public void execute() {
        hanger.frontClimberSetSpeed(elevationSpeed*GearboxRatio);
        SmartDashboard.putNumber("Percent to Max", ((hanger.getFrontClimberEncoderValue() / (2048 * GearboxRatio
         * Math.PI * Units.inchesToMeters(pulleyDiameter)) / maxFrontClimberEncoderValue) * 100));
    }
    @Override
  public void end(boolean interrupted) {
      //hanger.moveUpDown(motor, 0);
      hanger.frontClimberSetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (hanger.getFrontClimberEncoderValue() / (2048 * GearboxRatio
    * Math.PI * Units.inchesToMeters(pulleyDiameter)) / maxFrontClimberEncoderValue) > 0.9;
  }
}
