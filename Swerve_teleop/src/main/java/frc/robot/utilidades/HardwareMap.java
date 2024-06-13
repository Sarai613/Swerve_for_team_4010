package frc.robot.utilidades;

import frc.robot.swerve.SwerveModule;
public class HardwareMap{

    //Defines every single module by giving the drive spark id, the turning spark id, the absolute encoder id, absolute encoder offset, is inverted
    public static final SwerveModule frontLeft = new SwerveModule(23, 32, 13, Math.PI * 3 / 4 + .358, false);
    public static final SwerveModule frontRight = new SwerveModule(22, 24, 12, Math.PI * 3 / 4 - .166, true);
    public static final SwerveModule backLeft = new SwerveModule(21, 31, 10,  Math.PI / 2 - 0.162, true );
    public static final SwerveModule backRight = new SwerveModule(33, 34, 11,  Math.PI / 2, false);
}