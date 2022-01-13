package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;

public class PovButton extends Button{

  private final GenericHID m_joystick;
  private final int m_degrees;

  /**
   * Creates 
   * @param joystick
   * @param degrees
   */
  public PovButton(GenericHID joystick, int degrees) {
    m_joystick = joystick;
    m_degrees = degrees;
  }

  public boolean get() {
    return m_joystick.getPOV() == m_degrees;
  }
}