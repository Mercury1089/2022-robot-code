/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/* this is definently not a class to create a school shooter                  */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.CAN;
import frc.robot.sensors.Limelight;
import frc.robot.util.PIDGain;
import frc.robot.util.interfaces.IMercPIDTunable;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Shooter extends SubsystemBase implements IMercShuffleBoardPublisher, IMercPIDTunable {
  // private IMercMotorController flywheel;

  public static final double NOMINAL_OUT = 0.0, PEAK_OUT = 1.0;
  public static final double MIN_RPM = 3700.0, MAX_RPM = 5000.0, STEADY_RPM = 4000.0, LOW_RPM = 1000.0;

  private CANSparkMax shooterLeft, shooterRight;

  private double targetRPM;

  private ShooterMode mode;
  
  private PIDGain velocityGains;

  private Limelight limelight;

  DigitalInput breakBeamSensor;

  public enum ShooterMode {
    ONE_WHEEL, NONE
  }

  public Shooter(ShooterMode mode, Limelight limelight) {
    setName("Shooter");
    this.mode = mode;



    if (mode == ShooterMode.ONE_WHEEL) {
      shooterLeft = new CANSparkMax(CAN.SHOOTER_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
      shooterRight = new CANSparkMax(CAN.SHOOTER_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

      shooterLeft.getPIDController().setOutputRange(NOMINAL_OUT, PEAK_OUT);
      shooterRight.getPIDController().setOutputRange(NOMINAL_OUT, PEAK_OUT);

      shooterLeft.setIdleMode(IdleMode.kCoast);
      shooterRight.setIdleMode(IdleMode.kCoast);

      shooterLeft.setInverted(true);
      shooterRight.follow(shooterLeft, true); // Follow inverted
    } else if (mode == ShooterMode.NONE) {
      shooterLeft = shooterRight = null;
    }


    SmartDashboard.putNumber(getName() + "/SetRPM", 0.0);

    stopShooter();
    targetRPM = 0.0;
    velocityGains = new PIDGain(1e-5, 2e-7, 1e-5, 0);
    
    this.limelight = limelight;
  
    setPIDGain(SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), velocityGains);

    this.breakBeamSensor = new DigitalInput(2); 
  }

  public void stopShooter() {
    shooterLeft.stopMotor();
  }

  public double getRPM() {
    return shooterLeft != null ? shooterLeft.getEncoder().getVelocity() : 0.0;
  }

  public boolean isReadyToShoot(){
    return atTargetRPM();
  }

  public double getTargetRPM() {
    double distance = limelight.getDistanceToTarget();
    updateTargetRPMCenter(distance);
    return targetRPM < MAX_RPM && targetRPM > MIN_RPM ? targetRPM : STEADY_RPM;
    //return getRunRPM();
  }

  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
  }

  public double updateTargetRPMCenter(double distance) {
    targetRPM = 2932 * Math.exp(0.0246*distance);
    //targetRPM = -2.93032197e-09*Math.pow(distance, 6) + 3.21815380e-06*Math.pow(distance, 5) - 1.40572567e-03*Math.pow(distance, 4) + 3.06747428e-01*Math.pow(distance, 3) - 3.38724423e+01*Math.pow(distance, 2) + 1.60699276e+03*distance - 9.44326999e+03;
    return targetRPM;
  }

  public boolean atTargetRPM() {
    return Math.abs(getRPM() - getTargetRPM()) <= 0.01 * getTargetRPM();
  }

  public void setVelocity(double rpm) {
    if (shooterLeft != null && shooterRight != null)
    {
      // Sets RPM
      shooterLeft.getPIDController().setReference(rpm, ControlType.kVelocity);
    }
  }

  public double getRunRPM() {
    return SmartDashboard.getNumber(getName() + "/SetRPM", 0.0);
  }

  public boolean hasBall() {
    // if the shooter has a ball (if beam is broken)
    return !breakBeamSensor.get();
  }

  public void shootNow() {
    updateTargetRPMCenter(limelight.getDistanceToTarget());
    setVelocity(targetRPM);
  }

 

  public void publishValues() {
    SmartDashboard.putNumber(getName() + "/RPM", getRPM());
    
    //SmartDashboard.putNumber(getName() + "/PIDGains/P", velocityGains.kP);
    //SmartDashboard.putNumber(getName() + "/PIDGains/I", velocityGains.kI);
    //SmartDashboard.putNumber(getName() + "/PIDGains/D", velocityGains.kD);
    //SmartDashboard.putNumber(getName() + "/PIDGains/F", velocityGains.kF);

    SmartDashboard.putBoolean(getName() + "/AtTargetRPM", atTargetRPM());
    SmartDashboard.putNumber(getName() + "/TargetRPM", targetRPM);
    //SmartDashboard.putNumber("Hypothetical Distance", getHyotheticalDistance());
    //SmartDashboard.putNumber("Hypothetical RPM", getTargetRPMFromHypothetical());
  }

  
  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.setActuator(true); // Only allow setting values when in Test mode
    builder.addStringProperty("ShooterHasBall", () -> hasBall() + "", null);

    //builder.addDoubleProperty(key, getter, setter);
    
  }

  @Override
  public PIDGain getPIDGain(int slot) {
    return this.velocityGains;
  }

  private void configPID(CANSparkMax sparkmax, int slot, PIDGain gains) {
    sparkmax.getPIDController().setP(gains.kP, slot);
    sparkmax.getPIDController().setI(gains.kI, slot);
    sparkmax.getPIDController().setD(gains.kD, slot);
    sparkmax.getPIDController().setFF(gains.kF, slot);
  }

  @Override
  public void setPIDGain(int slot, PIDGain gains) {
    this.velocityGains = gains;

    if (shooterLeft != null && shooterRight != null) {
      configPID(shooterLeft, SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), this.velocityGains);
      configPID(shooterLeft, SHOOTER_PID_SLOTS.VELOCITY_GAINS.getValue(), this.velocityGains);
    }
  }



 

  

  @Override
  public int[] getSlots() {
    return new int[] { 0 };
  }

  public enum SHOOTER_PID_SLOTS {
    VELOCITY_GAINS(0);

    private int value;

    SHOOTER_PID_SLOTS(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }

    
  }
}
