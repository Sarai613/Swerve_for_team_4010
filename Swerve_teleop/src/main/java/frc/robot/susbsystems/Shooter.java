package frc.robot.susbsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilidades.Constants;

public class Shooter extends SubsystemBase {

  private final TalonSRX reloader = new TalonSRX(1);
  private final CANSparkMax launcher = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax launcher_1 = new CANSparkMax(5, MotorType.kBrushless);
  private final TalonSRX rotorMotor = new TalonSRX(2);
  private final TalonSRX rotorMotor_2 = new TalonSRX(3);
  private final DutyCycleEncoder rotorEncoder = new DutyCycleEncoder(Constants.HardwareMap.ROTOR_ENCODER_ID);
  private final PIDController rotatorPIDController = new PIDController(1, 0.2, 15);
  private final SlewRateLimiter lRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotatoRateLimiter = new SlewRateLimiter(3);

  /** Creates a new Shooter. */
  public Shooter() {
    rotorEncoder.setDistancePerRotation(Constants.ROTOR_ENCODER_POSITION_2_RADIAN);
  }

  public void setReloader(double power){
    reloader.set(TalonSRXControlMode.PercentOutput, rRateLimiter.calculate(power));
  }

  public void setLauncher(double power){
    launcher.set(lRateLimiter.calculate(power));
    launcher_1.set(lRateLimiter.calculate(power));
  }

  public void setLauncherVelocity(double velocity){
    if (velocity > Constants.LAUNCHER_MAX_VELOCITY){
      SmartDashboard.putString("Launcher", "La velocidad solicitada es mayor a la velocidad máxima");
      return;
    } 
    double power = lRateLimiter.calculate(velocity / Constants.LAUNCHER_MAX_VELOCITY);
    launcher.set(power);
    launcher_1.set(power);
  }

  public double getLauncherVelocity(){
    return launcher.get() * Constants.LAUNCHER_MAX_VELOCITY;
  }

  public double[] getRotatorPosition(){
    return new double[] {rotorMotor.getSelectedSensorPosition(), rotorMotor_2.getSelectedSensorPosition()};
  }

  public Rotation2d getAngle(){
    return new Rotation2d(rotorEncoder.getDistance());
  }

  public void setAngle(Rotation2d angle){ 
    while (Math.abs(getAngle().getRadians() - angle.getRadians()) >= 0.006){
      double power = rotatorPIDController.calculate(getAngle().getRadians(), angle.getRadians());
      rotorMotor.set(TalonSRXControlMode.PercentOutput, power);
      rotorMotor_2.set(TalonSRXControlMode.PercentOutput, power);
    }
  }

  public void setRotatorPower(double power){
    rotorMotor.set(TalonSRXControlMode.PercentOutput, rotatoRateLimiter.calculate(power));
    rotorMotor_2.set(TalonSRXControlMode.PercentOutput, rotatoRateLimiter.calculate(power));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
