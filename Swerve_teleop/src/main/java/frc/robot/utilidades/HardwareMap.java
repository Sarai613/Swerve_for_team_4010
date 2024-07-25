package frc.robot.utilidades;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.swerve.SwerveModule;
public class HardwareMap{

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    public static final SwerveModule frontLeft = new SwerveModule(33, 17, 2546, Math.PI * 3 / 4 + .358, false);
    public static final SwerveModule frontRight = new SwerveModule(31, 22, 39, Math.PI * 3 / 4 - .166, true);
    public static final SwerveModule backLeft = new SwerveModule(16, 18, 1513,  Math.PI / 2 - 0.162, true );
    public static final SwerveModule backRight = new SwerveModule(21, 34, 6126,  Math.PI / 2, false);
    public static final CANSparkMax intake_front_wheel = new CANSparkMax(24, MotorType.kBrushless);
    public static final CANSparkMax intake_back_wheel = new CANSparkMax(23, MotorType.kBrushless);
    public static final TalonSRX shooter_launcher = new TalonSRX(4);
    public static final TalonSRX shooter_reloader = new TalonSRX(1);
    public static final TalonSRX shooter_spin_motor = new TalonSRX(2);
    public static final TalonSRX shooter_spin_motor_2 = new TalonSRX(3);
    
}