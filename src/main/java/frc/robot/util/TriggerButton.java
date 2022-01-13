package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class TriggerButton extends Button{

  private static final double DEFAULT_THRESHOLD = 0.5;

  private final GenericHID m_joystick;
  private final int m_axisNumber;
  private final boolean m_polarity;
  private final double m_threshold;

  /**
   * Create a gamepad axis for triggering commands as if it were a button.
   *
   * @param joystick The GenericHID object that has the axis (e.g. Joystick, KinectStick, etc)
   * 
   * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
   * 
   * @param polarity The polarity of the axis (to specify which direction is considered the positive direction)
   * 
   * @param threshold The threshold above which the axis shall trigger a command
   */
  public TriggerButton(GenericHID joystick, int axisNumber, boolean polarity, double threshold) {
    m_joystick = joystick;
    m_axisNumber = axisNumber;
    m_polarity = polarity;
    m_threshold = threshold;
  }

  public TriggerButton(GenericHID joystick, int axisNumber, boolean polarity) {
    this(joystick, axisNumber, polarity, DEFAULT_THRESHOLD);
  }

  public TriggerButton(GenericHID joystick, int axisNumber) {
    this(joystick, axisNumber, true);
  }

  /**
   * Gets the value of the gamepad axis.
   *
   * @return The value of the gamepad axis
   */
  public boolean get() {
    double rawAxis = m_joystick.getRawAxis(m_axisNumber);

    if (m_polarity) {
			return (Math.max(rawAxis,0) > m_threshold);				
    } 
    else  {
			return (-Math.min(rawAxis,0) > m_threshold);
		}
  }
}