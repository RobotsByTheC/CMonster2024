package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Generic interface for reading data from an LED buffer. */
public interface LEDReader {
  /**
   * Gets the length of the buffer.
   *
   * @return the buffer length
   */
  int getLength();

  /**
   * Gets the most recently written color for a particular LED in the buffer.
   *
   * @param index the index of the LED
   * @return the LED color
   * @throws IndexOutOfBoundsException if the index is negative or greater than {@link #getLength()}
   */
  Color getLED(int index);

  /**
   * Gets the most recently written color for a particular LED in the buffer.
   *
   * @param index the index of the LED
   * @return the LED color
   * @throws IndexOutOfBoundsException if the index is negative or greater than {@link #getLength()}
   */
  Color8Bit getLED8Bit(int index);

  /**
   * Creates an {@code LEDReader} that wraps an addressable LED buffer, for compatibility with
   * animations.
   *
   * @param buffer the buffer to wrap
   * @return the wrapper
   */
  // TODO: AddressableLEDBuffer should implement this interface... we shouldn't need to wrap it
  static LEDReader forBuffer(AddressableLEDBuffer buffer) {
    return new LEDReader() {
      @Override
      public int getLength() {
        return buffer.getLength();
      }

      @Override
      public Color getLED(int index) {
        return buffer.getLED(index);
      }

      @Override
      public Color8Bit getLED8Bit(int index) {
        return buffer.getLED8Bit(index);
      }
    };
  }
}
