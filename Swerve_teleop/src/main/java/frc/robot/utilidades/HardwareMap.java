package frc.robot.utilidades;

import frc.robot.swerve.SwerveModule;
public class HardwareMap {
    public static final SwerveModule frontLeft = new SwerveModule(23, 32);
    public static final SwerveModule frontRight = new SwerveModule(22, 24);
    public static final SwerveModule backLeft = new SwerveModule(21, 31);
    public static final SwerveModule backRight = new SwerveModule(33, 34);
}