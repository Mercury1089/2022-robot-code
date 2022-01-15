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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap.CAN;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class Intake extends SubsystemBase implements IMercShuffleBoardPublisher {
  private final VictorSPX intakeRoller, agitator;
  public final double INTAKE_SPEED, AGITATOR_SPEED;
  public final boolean IS_CLOCKWISE;
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
    intakeRoller.setInverted(true);
    agitator = new VictorSPX(CAN.AGITATOR);
    agitator.setInverted(IS_CLOCKWISE);
    agitator.setNeutralMode(NeutralMode.Brake);
  }

  public void setRollerSpeed(double speed) {
    intakeRoller.set(ControlMode.PercentOutput, speed);
  }

  public void runIntakeRoller(double velocityProportion) {
    setRollerSpeed(INTAKE_SPEED * velocityProportion);
  }

  public void runIntakeRoller() {
    setRollerSpeed(INTAKE_SPEED);
  }

  public void stopIntakeRoller() {
    setRollerSpeed(0.0);
  }

  public void runAgitator() {
    agitator.set(ControlMode.PercentOutput, AGITATOR_SPEED);
  }

  public void stopAgitator() {
    agitator.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean getIsClockwise() {
    return IS_CLOCKWISE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void publishValues() {
    
  }
}
