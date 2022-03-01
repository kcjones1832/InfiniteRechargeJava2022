package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    //private final double AREA_DISTANCE_RATIO = 4.1823379; not used
    private final double MAX_STEER = 0.3;
    private final double STEER_K = 0.075;
    private final double DRIVE_K = 1.75;
    private final double MAX_DRIVE = 0.5;

    private Joystick stick = new Joystick(0);

    private boolean imageSwitch = false;

    private double rawDistance;

    private double turnOutput;
    private double moveOutput;
    private double moveWant = 2.34; //0.88 trench

    private double targetOffsetAngle_Horizontal;
    private double targetOffsetAngle_Vertical;
    private double targetArea;
    private double targetSkew;
    private double tv; //has target

    public void display() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry ts = table.getEntry("ts");

        targetOffsetAngle_Horizontal = tx.getDouble(0.0);
        targetOffsetAngle_Vertical = ty.getDouble(0.0);
        targetArea = ta.getDouble(0.0);
        targetSkew = ts.getDouble(0.0);

        SmartDashboard.putNumber("tx", targetOffsetAngle_Horizontal);
        SmartDashboard.putNumber("ty", targetOffsetAngle_Vertical);
        SmartDashboard.putNumber("ta", targetArea);
        SmartDashboard.putNumber("ts", targetSkew);

        table.getEntry("pipeline").setNumber(3);
        //table->PutNumber("pipeline", 3);

        if (stick.getRawButtonPressed(4)) {
            imageSwitch = !imageSwitch;
        }

        if (imageSwitch) {
            table.getEntry("camMode").setNumber(0);
            table.getEntry("ledMode").setNumber(3);
        }
        else {
            table.getEntry("camMode").setNumber(1);
            table.getEntry("ledMode").setNumber(1);
        }

    }

    public double distance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        targetArea = table.getEntry("ta").getDouble(0.0);

        rawDistance = (targetArea * -4752.0) + 121947; //* -4752.0) + 124947
        SmartDashboard.putNumber("velocity calc", rawDistance);

        return rawDistance;
    }

    private double clamp(double in,double minval,double maxval) {
        if (in > maxval) {
          return maxval;
        }
        else if (in < minval) {
          return minval;
        }
        else {
          return in;
        }
    }

    public double trackTurn() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);

        if (tv == 1) {
            turnOutput = targetOffsetAngle_Horizontal * STEER_K; //or divid by max value (27 degrees)
            turnOutput = clamp(turnOutput,-MAX_STEER,MAX_STEER);
            return turnOutput;
        }
        else {
            return 0;
        }
    }

    public double trackMove() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        targetArea = table.getEntry("ta").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);
      
        moveWant = SmartDashboard.getNumber("ta want", 2.34);
      
        if (tv == 1) {
          moveOutput = (targetArea - moveWant) * DRIVE_K;
          moveOutput = clamp(moveOutput, -MAX_DRIVE,MAX_DRIVE);
          return -moveOutput;
        }
        else {
          return 0;
        }
    }
}
