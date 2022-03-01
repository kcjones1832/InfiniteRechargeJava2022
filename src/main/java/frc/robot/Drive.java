package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
    private CANSparkMax driveMotorLeft = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax driveMotorRight = new CANSparkMax(3, MotorType.kBrushless);

    private CANSparkMax slaveMotorLeft1 = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax slaveMotorLeft2 = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax slaveMotorRight1 = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax slaveMotorRight2 = new CANSparkMax(7, MotorType.kBrushless);

    private DifferentialDrive robotDrive = new DifferentialDrive(driveMotorLeft, driveMotorRight);
    private Joystick stick = new Joystick(0);
    //private XboxController xbox = new XboxController(1); not currently used

    private AHRS gyro;

    private double xStickValue; 
    private double yStickValue;

    //for auto later
    /*private double revNeed;
    private double offset;
    private double leftCurrentPos;
    private double rightCurrentPos;
    private double avgPosition;
    private double power;
    private double leftEncLast;
    private double rightEncLast;
    private double gyroLast;
    private double turnCorrection;
    private double turnOffset;*/

    double currentAngle;

    public Drive() {
        driveMotorRight.setInverted(true);

        slaveMotorLeft1.follow(driveMotorLeft);
        slaveMotorLeft2.follow(driveMotorLeft);
        slaveMotorRight1.follow(driveMotorRight);
        slaveMotorRight2.follow(driveMotorRight);

        driveMotorLeft.setSmartCurrentLimit(60); //sets max current limit
        driveMotorRight.setSmartCurrentLimit(60); //defalt is 80
        slaveMotorLeft1.setSmartCurrentLimit(60);
        slaveMotorLeft2.setSmartCurrentLimit(60);
        slaveMotorRight1.setSmartCurrentLimit(60);
        slaveMotorRight2.setSmartCurrentLimit(60); 

        driveMotorLeft.getEncoder().setPosition(0);
        driveMotorRight.getEncoder().setPosition(0);

        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            gyro = new AHRS(Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        gyro.reset();
    }

    private int Sign(double input) { //method that returns numbers for its relation to 0
        if (input > 0) {
            return 1;
        }
        else if (input < 0) {
            return -1;
        }
        else if (input == 0) {
            return 0;
        }
        else {
            return 0;
        }
    }
    
    private double deadband(double joystickValue, double deadbandValue) { //colins special proportioanl deadband thingy just copy
        if(Math.abs(joystickValue) < 0.2){
            return 0;
        }
        else{
            return (1 / (1 - deadbandValue)) * (joystickValue + (-Sign(joystickValue) * deadbandValue));
        } 
    }

    public void drive() {
        xStickValue = -deadband(stick.getRawAxis(1), 0.2); //getting raw axis values
        yStickValue = deadband(stick.getRawAxis(2), 0.2);

        if(stick.getRawButton(1)){
            xStickValue *= 0.70;
            yStickValue *= 0.55;
        }
          if (stick.getRawAxis(3) > 0.9) {
            xStickValue *= 0.75;
            yStickValue *= 0.6;
        }
        if (stick.getRawButton(2)) {
            xStickValue *= 0.50;
            yStickValue *= 0.45;
        }
        
        robotDrive.arcadeDrive(xStickValue, yStickValue);
        
        SmartDashboard.putNumber("left encoder", driveMotorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("right encoder", driveMotorRight.getEncoder().getPosition());

        SmartDashboard.putNumber("left current", driveMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("right current", driveMotorRight.getOutputCurrent());
        SmartDashboard.putNumber("left Temp", driveMotorLeft.getMotorTemperature());
        SmartDashboard.putNumber("right Temp", driveMotorRight.getMotorTemperature());

        if (stick.getRawButton(5)) {
            gyro.reset();
        }
        SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    }

    public void subclassTurn(double turnValue, double moveValue) {
        robotDrive.arcadeDrive(moveValue, turnValue);
    }
}
