package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
    ///private Joystick stick = new Joystick(0); not used
    private XboxController xbox = new XboxController(1);
    
    private CANSparkMax intakeRotateMotor = new CANSparkMax(10, MotorType.kBrushless);
    private WPI_TalonSRX intakeSpinMotor = new WPI_TalonSRX(11);

    private int rotateControlMode = 0;
    private double rotatePosition;

    final double LOW_GOAL_HEIGHT = 7.2;
    
    public Manipulator() {
        intakeRotateMotor.getEncoder().setPosition(0);
    }

    public void intake() {
        if (xbox.getRawButton(1)) {
            intakeSpinMotor.set(-0.8);
        }
        else if (xbox.getRawButton(2)) {
            intakeSpinMotor.set(0.8);
        }
        else {
            intakeSpinMotor.set(0);
        }

        if (xbox.getPOV() == 180) {
            rotateControlMode = 2;
        }
        else if (xbox.getPOV() == 270) {
            rotateControlMode = 3;
        }
        else if (xbox.getPOV() == 90) {
            rotateControlMode = 4;
        }

        rotatePosition = intakeRotateMotor.getEncoder().getPosition();
        if (rotateControlMode == 1) {
            if (rotatePosition > LOW_GOAL_HEIGHT) {
                intakeRotateMotor.set(-0.022);
            }
            else if (rotatePosition < (LOW_GOAL_HEIGHT - 0.4)){
                intakeRotateMotor.set(0.03);
            }
        }
        else if (rotateControlMode == 2) {
            if (rotatePosition < 42 && rotatePosition > 20) { //replace 0 with desired location
                intakeRotateMotor.set(-0.00005);
            }
            else if (rotatePosition < 16) {
                intakeRotateMotor.set(0.04);
            }
            else if (rotatePosition < 19) {
                intakeRotateMotor.set(-0.000);
            }
            else {
                intakeRotateMotor.set(0);
            }
        }
        if (rotateControlMode == 3) {
            /*if (rotatePosition > 45 && xbox.GetRawAxis(5) > 0) {
              intakeRotateMotor.set(0);
            }
            else {
              intakeRotateMotor.set(xbox.GetRawAxis(5) * 0.25);
            }*/
            if (rotatePosition > 14) {
              intakeRotateMotor.set(-0.19);
            }
            else {
              intakeRotateMotor.set(-0.16);
            }
            
        
            if (rotatePosition < LOW_GOAL_HEIGHT) {
              rotateControlMode = 1;
            }
        }
        if (rotateControlMode == 4) {
            intakeRotateMotor.set(xbox.getRawAxis(5) * 0.4);
        }

        SmartDashboard.putNumber("intake rotate encoder", intakeRotateMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("intake rotate power", intakeRotateMotor.get());
        SmartDashboard.putNumber("intake rotate current", intakeRotateMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake rotate temp", intakeRotateMotor.getMotorTemperature());
        SmartDashboard.putNumber("rotate control mode", rotateControlMode);
    }

    void intakeStartup() {

        rotatePosition = intakeRotateMotor.getEncoder().getPosition();
        if (rotatePosition < 42 && rotatePosition > 22) { //replace 0 with desired location
          intakeRotateMotor.set(-0.00005);
        }
        else if (rotatePosition > -1 && rotatePosition < 15) {
          intakeRotateMotor.set(0.08);
        }
        else if (rotatePosition < 20) {
          intakeRotateMotor.set(-0.000);
        }
        else {
          intakeRotateMotor.set(0);
        }
    
        if (rotatePosition > 42) {
          Robot.autoStep++;
        }
    
        //frc::SmartDashboard::PutNumber("intake rotate encoder", intakeRotateMotor->GetEncoder().GetPosition());
      //frc::SmartDashboard::PutNumber("intake rotate power", intakeRotateMotor->Get());
      //frc::SmartDashboard::PutNumber("intake rotate current", intakeRotateMotor->GetOutputCurrent());
      //frc::SmartDashboard::PutNumber("intake rotate temp", intakeRotateMotor->GetMotorTemperature());
    }
}
