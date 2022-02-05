package frc.robot.commands.turret;

import frc.robot.subsystems.Turret;

/**
 * https://youtu.be/NnP5iDKwuwk
 */
public class StayOnTarget extends RotateToTarget {

    public StayOnTarget(Turret turret) {
        super(turret);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}