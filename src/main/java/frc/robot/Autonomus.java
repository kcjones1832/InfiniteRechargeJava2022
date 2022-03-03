package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomus {
    Drive drive;
    Manipulator manipulator;
    Shooter shooter;
    int autoStep;
    Timer autoTime;

    public Autonomus(Drive drive, Manipulator manipulator, Shooter shooter) {
        this.drive = drive;
        this.manipulator = manipulator;
        this.shooter = shooter;

        autoTime = new Timer();
        autoTime.reset();
    }

    void testAuto() {
        switch(Robot.autoStep) {
            case 0:
                manipulator.intakeStartup();
                break;
            case 1:
                drive.autoPrep();
                break;
            case 2:
                drive.autoBasic();
                break;
        }
        SmartDashboard.putNumber("auto step", autoStep);
    }

    void shootMiddle(int foward1) {
        SmartDashboard.putNumber("auto step", autoStep);
    
        switch(Robot.autoStep) {
        case 0:
            manipulator.intakeStartup();
            drive.zeroDrive();
            break;
        case 1: 
            autoTime.start();
            drive.zeroDrive();
            Robot.autoStep++;
            break;
        case 2:
            shooter.shootAuto(true);
            drive.zeroDrive();
            //manipulator.autoTimer(4);
            if (autoTime.get() > 4) {
                autoTime.stop();
                Robot.autoStep++;
            }
            break;
        case 3:
            shooter.stopShoot();
            break;
        case 4:
            drive.autoPrep();
            break;
        case 5:
            drive.autoDrive(3 * foward1);
            break;
        case 6:
            drive.zeroDrive();
            break;
        }
    }
}
