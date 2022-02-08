package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LimeConstants.*;
import static frc.robot.Constants.LimelightConstants.*;

public class LimeLight extends SubsystemBase {

    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta, tv, ts;
    private double x, y, area, valid, skew;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        ts = table.getEntry("ts");
    }

    public void update() {
        x = tx.getDouble(100.0);
        y = ty.getDouble(100.0);
        area = ta.getDouble(100.0);
        valid = tv.getDouble(100.0);
        skew = ts.getDouble(100.0);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightValid", valid);
        SmartDashboard.putNumber("LimelightSkew", skew);
    }

    public double getTx() {
        update();
        return x;
    }

    public double getTv() {
        update();
        return valid;
    }

    public double getDistaceToTarget() {
        update();
        double distance = ((targetHeight - limelightHeight ) / Math.tan((ANGLE+y)*(Math.PI/180))+24);
        return distance;
    }
    

}