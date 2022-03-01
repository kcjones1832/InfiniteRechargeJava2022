package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private WPI_TalonSRX shootMotor = new WPI_TalonSRX(12);
    private WPI_TalonSRX feederMotor = new WPI_TalonSRX(13);
    private WPI_TalonSRX hoodMotor = new WPI_TalonSRX(14);

    private XboxController xbox = new XboxController(1);

    private AnalogPotentiometer hoodPotent = new AnalogPotentiometer(1, 1000, 0);

    private int hoodPosition;
    private double velocityWant;

    public Shooter() {
        shootMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        shootMotor.setSensorPhase(false);
        shootMotor.setInverted(false);
        shootMotor.config_kP(0, 0.06, 10);
        shootMotor.config_kI(0, 0.06, 10);
        shootMotor.config_kD(0, 0.06, 10);
        shootMotor.setNeutralMode(NeutralMode.Coast);
        shootMotor.configPeakOutputForward(0, 10);
        shootMotor.configClosedloopRamp(1, 10);
    }

    private void hoodRotate(double hoodPositionWant) {
        hoodPosition = (int)hoodPotent.get();
        hoodMotor.set((hoodPosition - hoodPositionWant) * 0.055);
    }

    public void shootTest(double in) {
        if (xbox.getRawAxis(2) > 0.9) {
            hoodMotor.set(0.5);
        }
        else if (xbox.getRawAxis(3) > 0.9) {
            hoodMotor.set(-0.5);
        }
        else {
            hoodMotor.set(0);
        }

        if (xbox.getRawButton(5)) {
            hoodRotate(30.0);
        }

        SmartDashboard.putNumber("hood position", hoodPotent.get());

        velocityWant = SmartDashboard.getNumber("shoot position", 87500); //97000

        if (xbox.getRawButton(4)) {
            //shootMotor->Set((velocityWant * 1.0) / -120000.0);
            shootMotor.set(ControlMode.Velocity, -velocityWant);
        }
        else if (xbox.getRawButton(8)) {
            shootMotor.set(ControlMode.Velocity, -in);
        }
        else if (xbox.getRawButton(3)) {
            shootMotor.set(1.1 * xbox.getRawAxis(1));
        }

        SmartDashboard.putNumber("shooter velocity", shootMotor.getSensorCollection().getQuadratureVelocity());
        SmartDashboard.putNumber("shoot power", shootMotor.get());
        SmartDashboard.putNumber("shoot voltage", -shootMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("shooter current", shootMotor.getStatorCurrent());

        if (xbox.getPOV() == 0) {
            feederMotor.set(-0.8);
        }
        else {
            feederMotor.set(0);
        }
    }
}
