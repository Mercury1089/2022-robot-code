package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.RobotMap.CAN;

public class ClimberWinch extends SubsystemBase{
    private final TalonFX climberWinch;
    private ClimberWinchStatus climberWinchStatus;
    private double CLIMBER_WINCH_SPEED = 0.5;

    public ClimberWinch() {
        super();
        climberWinch = new TalonFX(CAN.CLIMBER_WINCH);
        climberWinchStatus = ClimberWinchStatus.DISABLED;
    }

    public enum ClimberWinchStatus {
        ACTIVE,
        DISABLED
    }

    public void setClimberWinchSpeedActive() {
        this.climberWinchStatus = ClimberWinchStatus.ACTIVE;
        climberWinch.set(ControlMode.PercentOutput,CLIMBER_WINCH_SPEED);
    }

    public void setClimberWinchSpeedDisabled() {
        this.climberWinchStatus = ClimberWinchStatus.DISABLED;
        CLIMBER_WINCH_SPEED = 0.0;
        climberWinch.set(ControlMode.PercentOutput,CLIMBER_WINCH_SPEED);
    }

    public ClimberWinchStatus getClimberWinchStatus() {
        return this.climberWinchStatus; 
    }

    public double getClimberWinchSpeed() {
        return CLIMBER_WINCH_SPEED;
    }

    public void setSpeed(Supplier<Double> speed) {
        climberWinch.set(ControlMode.PercentOutput, speed.get());
    }

}

