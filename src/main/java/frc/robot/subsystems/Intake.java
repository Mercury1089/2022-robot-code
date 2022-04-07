/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;

public class Intake extends SubsystemBase {

  public enum IntakeSpeed{
    STOP(0.0), // stop running intake roller
    INTAKE(1.0), // run intake roller inward
    EJECT(-1.0); // run intake roller outward

    public final double speed; 

        /**
         * Creates the intake speed.
         *
         * @param speed speed, in PercentOutput
         */
        IntakeSpeed(double speed) {
            this.speed = speed;
        }
  }

  private final VictorSPX intakeRoller;
  public final double INTAKE_SPEED, AGITATOR_SPEED;
  public final boolean IS_CLOCKWISE;
  private boolean smartdashIntakeFullSpeed = false;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    super();

    INTAKE_SPEED = 1.0;
    AGITATOR_SPEED = 0.5;
    IS_CLOCKWISE = true;
    
    setName("Intake");
    
    intakeRoller = new VictorSPX(CAN.INTAKE_ROLLER);
    intakeRoller.configFactoryDefault();
    intakeRoller.setNeutralMode(NeutralMode.Brake);
    intakeRoller.setInverted(true);
  }

  public void setSpeed(IntakeSpeed intakeSpeed) {
    intakeRoller.set(ControlMode.PercentOutput, intakeSpeed.speed);
  }

  public boolean getIsClockwise() {
    return IS_CLOCKWISE;
  }

  public boolean getIntakeFullSpeed() {
    return this.smartdashIntakeFullSpeed;
  }

  public void setIntakeFullSpeed(boolean intakeFullSpeed) {
    this.smartdashIntakeFullSpeed = intakeFullSpeed;
    if (smartdashIntakeFullSpeed) {
      setSpeed(IntakeSpeed.INTAKE);
    } else {
      setSpeed(IntakeSpeed.STOP);
    }
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Run Intake", () -> getIntakeFullSpeed(), (x) -> setIntakeFullSpeed(x));
  }


}
