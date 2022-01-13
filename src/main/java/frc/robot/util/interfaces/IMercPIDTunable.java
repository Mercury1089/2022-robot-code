package frc.robot.util.interfaces;

import frc.robot.util.PIDGain;

public interface IMercPIDTunable {
    public int[] getSlots();
    public PIDGain getPIDGain(int slot);
    public void setPIDGain(int slot, PIDGain gains);
}