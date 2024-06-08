package frc.robot.utilidades;

import frc.robot.swerve.SwerveModule;
public class HardwareMap{

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset
    public static final SwerveModule frontLeft = new SwerveModule(23, 32, 20, 0, true);
    public static final SwerveModule frontRight = new SwerveModule(22, 24, 21, 0, false);
    public static final SwerveModule backLeft = new SwerveModule(21, 31, 22, 0, false );
    public static final SwerveModule backRight = new SwerveModule(33, 34, 23, 0, true);
}