package frc.robot.Utils;

import frc.robot.swerve.SwerveModule;
public class HardwareMap {
    public static final SwerveModule frontLeft = new SwerveModule(23, 32);
    public static final SwerveModule frontRight = new SwerveModule(24, 22);
    public static final SwerveModule backLeft = new SwerveModule(31, 21);
    public static final SwerveModule backRight = new SwerveModule(14, 33);
}