package frc.robot.utils;

import frc.robot.swerve.SwerveModule;

public class HardwareMap {
    public static final SwerveModule frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
    public static final SwerveModule frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
    public static final SwerveModule backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
    public static final SwerveModule backRight = new SwerveModule(7, 8, 12, 13, 14, 15);
}