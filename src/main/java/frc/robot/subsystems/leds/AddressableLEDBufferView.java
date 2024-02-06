package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * A view of another addressable LED buffer. Views CANNOT be written directly to an LED strip; the
 * backing buffer must be written instead. However, views provide an easy way to split a large LED
 * strip into smaller sections (which may be reversed from the orientation of the LED strip as a
 * whole) that can be animated individually without modifying LEDs outside those sections.
 *
 * <pre><code>
 *   AddressableLED leds = new AddressableLED(0);
 *   AddressableLEDBuffer buffer = new AddressableLEDBuffer(120);
 *   AddressableLEDBufferView leftLEDs = new AddressableLEDBufferView(buffer, 0, 59);
 *   AddressableLEDBufferView rightLEDs = new AddressableLEDBufferView(buffer, 60, 119).reversed();
 *
 *   Animation leftAnimation = ...
 *   Animation rightAnimation = ...
 *
 *   void update() {
 *     // update the left and right sides individually...
 *     leftAnimation.write(leftLEDs);
 *     rightAnimation.write(rightLEDs);
 *
 *     // ... then write the combined data out to the LED strip
 *     leds.writeData(buffer);
 *   }
 * </code></pre>
 */
public class AddressableLEDBufferView extends AddressableLEDBuffer implements LEDReader, LEDWriter {
  private final AddressableLEDBuffer backingBuffer;
  private final int length;
  private final int startingIndex;
  private final int endingIndex;

  /**
   * Creates a new view of a buffer. A view will be reversed if the starting index is after the
   * ending index; writing front-to-back in the view will write in the back-to-front direction on
   * the underlying buffer.
   *
   * @param backingBuffer the backing buffer to view
   * @param startingIndex the index of the LED in the backing buffer that the view should start from
   * @param endingIndex the index of the LED in the backing buffer that the view should end on
   */
  public AddressableLEDBufferView(
      AddressableLEDBuffer backingBuffer, int startingIndex, int endingIndex) {
    super(0);
    if (startingIndex < 0 || startingIndex >= backingBuffer.getLength()) {
      throw new IndexOutOfBoundsException("Start index out of range: " + startingIndex);
    }
    if (endingIndex < 0 || endingIndex >= backingBuffer.getLength()) {
      throw new IndexOutOfBoundsException("End index out of range: " + endingIndex);
    }

    this.backingBuffer = backingBuffer;
    this.startingIndex = startingIndex;
    this.endingIndex = endingIndex;
    this.length = Math.abs(endingIndex - startingIndex) + 1;
  }

  public AddressableLEDBufferView reversed() {
    return new AddressableLEDBufferView(backingBuffer, endingIndex, startingIndex);
  }

  @Override
  public int getLength() {
    return length;
  }

  @Override
  public void setRGB(int index, int r, int g, int b) {
    backingBuffer.setRGB(nativeIndex(index), r, g, b);
  }

  @Override
  public Color getLED(int index) {
    return backingBuffer.getLED(nativeIndex(index));
  }

  @Override
  public Color8Bit getLED8Bit(int index) {
    return backingBuffer.getLED8Bit(nativeIndex(index));
  }

  /**
   * Checks if this view is reversed with respect to its backing buffer.
   *
   * @return true if the view is reversed, false otherwise
   */
  public boolean isReversed() {
    return endingIndex < startingIndex;
  }

  /**
   * Converts a view-local index in the range [start, end] to a global index in the range [0,
   * length].
   *
   * @param viewIndex the view-local index
   * @return the corresponding global index
   */
  private int nativeIndex(int viewIndex) {
    if (isReversed()) {
      //  0  1  2  3   4  5  6  7   8  9  10
      //  ↓  ↓  ↓  ↓   ↓  ↓  ↓  ↓   ↓  ↓  ↓
      // [_, _, _, _, (d, c, b, a), _, _, _]
      //               ↑  ↑  ↑  ↑
      //               3  2  1  0
      if (viewIndex < 0 || viewIndex > startingIndex) {
        throw new IndexOutOfBoundsException(viewIndex);
      }
      return startingIndex - viewIndex;
    } else {
      //  0  1  2  3   4  5  6  7   8  9  10
      //  ↓  ↓  ↓  ↓   ↓  ↓  ↓  ↓   ↓  ↓  ↓
      // [_, _, _, _, (a, b, c, d), _, _, _]
      //               ↑  ↑  ↑  ↑
      //               0  1  2  3
      if (viewIndex < 0 || viewIndex > endingIndex) {
        throw new IndexOutOfBoundsException(viewIndex);
      }
      return startingIndex + viewIndex;
    }
  }
}
