package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

@FunctionalInterface
public interface LEDWriter {
  /**
   * Sets the RGB value for an LED at a specific index on a LED buffer.
   *
   * @param index the index of the LED to write to
   * @param r the value of the red channel, in [0, 255]
   * @param g the value of the green channel, in [0, 255]
   * @param b the value of the blue channel, in [0, 255]
   */
  void setRGB(int index, int r, int g, int b);

  /**
   * Sets the RGB value for an LED at a specific index on a LED buffer.
   *
   * @param index the index of the LED to write to
   * @param color the color to set
   */
  default void setLED(int index, Color color) {
    setRGB(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets the RGB value for an LED at a specific index on a LED buffer.
   *
   * @param index the index of the LED to write to
   * @param color the color to set
   */
  default void setLED(int index, Color8Bit color) {
    setRGB(index, color.red, color.green, color.blue);
  }
}
