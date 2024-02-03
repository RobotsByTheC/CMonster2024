package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microsecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.collections.LongToObjectHashMap;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * An animation controls lights on an LED strip to command patterns of color that may change over
 * time.
 */
@FunctionalInterface
public interface Animation {
  /**
   * Writes the animation to an LED buffer. Dynamic animations should be called periodically (such
   * as with a command or with a periodic method) to refresh the buffer over time.
   *
   * <p>This method is intentionally designed to use separate objects for reading and writing data.
   * By splitting them up, we can easily modify the behavior of some base animation to make it
   * {@link #scrollAtRelativeSpeed(Measure) scroll}, {@link #blink(Measure, Measure) blink}, or
   * {@link #breathe(Measure) breathe}.
   *
   * @param reader data reader for accessing buffer length and current colors
   * @param writer data writer for setting new LED colors on the buffer
   */
  void update(LEDReader reader, LEDWriter writer);

  /**
   * Convenience for {@link #update(LEDReader, LEDWriter)} when one object provides both a read and
   * a write interface.
   *
   * @param readWriter the object to use for both reading and writing to a set of LEDs
   * @param <T> the type of the object that can both read and write data
   */
  default <T extends LEDReader & LEDWriter> void update(T readWriter) {
    update(readWriter, readWriter);
  }

  /**
   * Creates an animation that plays this one in reverse. Has no effect on static animations.
   *
   * @return the reverse animation
   */
  default Animation reversed() {
    return (reader, writer) -> {
      int bufLen = reader.getLength();
      update(reader, (i, r, g, b) -> writer.setRGB(bufLen - i, r, g, b));
    };
  }

  /**
   * Creates an animation that plays this one, but offset by a certain number of LEDs. The offset
   * animation will wrap around, if necessary.
   *
   * @param offset how many LEDs to offset by
   * @return the offset animation
   */
  default Animation offsetBy(int offset) {
    return (reader, writer) -> {
      int bufLen = reader.getLength();
      update(reader, (i, r, g, b) -> writer.setRGB((i + offset) % bufLen, r, g, b));
    };
  }

  /**
   * Creates an animation that plays this one scrolling up the buffer. The velocity controls how
   * fast the animation returns back to its original position, and is in terms of the length of the
   * LED strip; scrolling across a segment that is 10 LEDs long will travel twice as fast as on a
   * segment that's only 5 LEDs long (assuming equal LED density on both segments).
   *
   * @param velocity how fast the animation should move, in terms of how long it takes to do a full
   *     scroll along the length of LEDs and return back to the starting position
   * @return the scrolling animation
   */
  default Animation scrollAtRelativeSpeed(Measure<Velocity<Dimensionless>> velocity) {
    final double periodMicros = 1 / velocity.in(Value.per(Microsecond));

    return (reader, writer) -> {
      int bufLen = reader.getLength();
      long now = WPIUtilJNI.now();

      // index should move by (buf.length) / (period)
      double t = (now % (long) periodMicros) / periodMicros;
      int offset = (int) (t * bufLen);

      update(
          reader,
          (i, r, g, b) -> {
            // floorMod so if the offset is negative, we still get positive outputs
            int shiftedIndex = Math.floorMod(i + offset, bufLen);

            writer.setRGB(shiftedIndex, r, g, b);
          });
    };
  }

  /**
   * Creates an animation that plays this one scrolling up an LED strip. A negative velocity makes
   * the animation play in reverse.
   *
   * @param velocity how fast the animation should move along a physical LED strip
   * @param ledSpacing the distance between adjacent LEDs on the physical LED strip
   * @return the scrolling animation
   */
  default Animation scrollAtAbsoluteSpeed(
      Measure<Velocity<Distance>> velocity, Measure<Distance> ledSpacing) {
    var relativeVelocity =
        Value.per(Second).of(velocity.in(MetersPerSecond) * ledSpacing.in(Meters));
    return scrollAtRelativeSpeed(relativeVelocity);
  }

  /**
   * Creates an animation that switches between playing this animation and turning the entire LED
   * strip off.
   *
   * @param onTime how long the animation should play for, per cycle
   * @param offTime how long the animation should be turned off for, per cycle
   * @return the blinking animation
   */
  default Animation blink(Measure<Time> onTime, Measure<Time> offTime) {
    final long totalTimeMicros = (long) (onTime.in(Microseconds) + offTime.in(Microseconds));
    final long onTimeMicros = (long) onTime.in(Microseconds);

    return (reader, writer) -> {
      if (WPIUtilJNI.now() % totalTimeMicros < onTimeMicros) {
        update(reader, writer);
      } else {
        OFF.update(reader, writer);
      }
    };
  }

  /**
   * Like {@link #blink(Measure, Measure) blink(onTime, offTime)}, but where the "off" time is
   * exactly equal to the "on" time.
   *
   * @param onTime how long the animation should play for (and be turned off for), per cycle
   * @return the blinking animation
   */
  default Animation blink(Measure<Time> onTime) {
    return blink(onTime, onTime);
  }

  /**
   * Creates an animation that blinks this one on and off in sync with a true/false signal. The
   * animation will play while the signal outputs {@code true}, and will turn off while the signal
   * outputs {@code false}.
   *
   * @param signal the signal to synchronize with
   * @return the blinking animation
   */
  default Animation synchronizedBlink(BooleanSupplier signal) {
    return (reader, writer) -> {
      if (signal.getAsBoolean()) {
        update(reader, writer);
      } else {
        OFF.update(reader, writer);
      }
    };
  }

  /**
   * Creates an animation that brightens and dims this one over time. Brightness follows a
   * sinusoidal pattern.
   *
   * @param period how fast the breathing animation should complete a single cycle
   * @return the breathing animation
   */
  default Animation breathe(Measure<Time> period) {
    final long periodMicros = (long) period.in(Microseconds);

    return (reader, writer) -> {
      update(
          reader,
          (i, r, g, b) -> {
            // How far we are in the cycle, in the range [0, 1)
            double t = (WPIUtilJNI.now() % periodMicros) / (double) periodMicros;
            double phase = t * 2 * Math.PI;

            // Apply the cosine function and shift its output from [-1, 1] to [0, 1]
            // Use cosine so the period starts at 100% brightness
            double dim = (Math.cos(phase) + 1) / 2.0;

            // TODO: Avoid allocations here
            writer.setLED(i, lerp(new Color(r, g, b), Color.kBlack, dim));
          });
    };
  }

  /**
   * Creates an animation that plays this animation overlaid on another. Anywhere this animation
   * sets an LED to off (or {@link Color#kBlack}), the base animation will be displayed instead.
   *
   * @param base the base animation to overlay on top of
   * @return the combined overlay animation
   */
  default Animation overlayOn(Animation base) {
    return (reader, writer) -> {
      // write the base animation down first...
      base.update(reader, writer);

      // ... then, overwrite with the illuminated LEDs from the overlay
      update(
          reader,
          (i, r, g, b) -> {
            if (r != 0 || g != 0 || b != 0) {
              writer.setRGB(i, r, g, b);
            }
          });
    };
  }

  /**
   * Creates an animation that displays outputs as a combination of this animation and another.
   * Color values are calculated as the average color of both animations; if both animations set the
   * same LED to the same color, then it is set to that color, but if one animation sets to one
   * color and the other animation sets it to off, then it will show the color of the first
   * animation but at approximately half brightness. This is different from {@link #overlayOn},
   * which will show the base animation at full brightness if the overlay is set to off at that
   * position.
   *
   * @param other the animation to blend with
   * @return the blended animation
   */
  default Animation blend(Animation other) {
    return (reader, writer) -> {
      other.update(reader, writer);

      update(
          reader,
          (i, r, g, b) -> {
            // TODO: Avoid allocations here
            Color8Bit baseColor = reader.getLED8Bit(i);
            Color8Bit blendColor = lerp(baseColor, new Color8Bit(r, g, b), 0.5);

            writer.setLED(i, blendColor);
          });
    };
  }

  /** Displays a solid red color on all LEDs. */
  Animation SOLID_RED = solid(Color.kRed);

  /** Displays a solid blue color on all LEDs. */
  Animation SOLID_BLUE = solid(Color.kBlue);

  /** Turns off all LEDs. */
  Animation OFF = solid(Color.kBlack);

  /** Blinks between solid red and off, in time with the RSL. */
  Animation BLINK_RED = SOLID_RED.synchronizedBlink(HAL::getRSLState);

  /** Blinks between solid blue and off, in time with the RSL. */
  Animation BLINK_BLUE = SOLID_BLUE.synchronizedBlink(HAL::getRSLState);

  /**
   * Creates an animation that displays a single static color along the entire length of the LED
   * strip.
   *
   * @param color the color to display
   * @return the animation
   */
  static Animation solid(Color color) {
    return (reader, writer) -> {
      int bufLen = reader.getLength();
      for (int led = 0; led < bufLen; led++) {
        writer.setLED(led, color);
      }
    };
  }

  /**
   * Display a set of colors in steps across the length of the LED strip. No interpolation is done
   * between colors. Colors are specified by the first LED on the strip to show that color. The last
   * color in the map will be displayed all the way to the end of the strip. LEDs positioned before
   * the first specified step will be turned off (you can think of this as if there's a 0 -> black
   * step by default)
   *
   * <pre>
   *   // Display red from 0-33%, white from 33% - 67%, and blue from 67% to 100%
   *   steps(Map.of(0.00, Color.kRed, 0.33, Color.kWhite, 0.67, Color.kBlue))
   *
   *   // Half off, half on
   *   steps(Map.of(0.5, Color.kWhite))
   * </pre>
   *
   * @param steps a map of progress to the color to start displaying at that position along the LED
   *     strip
   * @return a static step animation
   */
  static Animation steps(Map<? extends Number, Color> steps) {
    if (steps.isEmpty()) {
      // no colors specified
      DriverStation.reportWarning("Creating LED steps with no colors!", false);
      return OFF;
    }

    if (steps.size() == 1 && steps.keySet().iterator().next().doubleValue() == 0) {
      // only one color specified, just show a static color
      DriverStation.reportWarning("Creating LED steps with only one color!", false);
      return solid(steps.values().iterator().next());
    }

    return (reader, writer) -> {
      int bufLen = reader.getLength();

      // precompute relevant positions for this buffer so we don't need to do a check
      // on every single LED index
      var stopPositions = new LongToObjectHashMap<Color>();
      steps.forEach(
          (progress, color) -> {
            stopPositions.put((int) Math.floor(progress.doubleValue() * bufLen), color);
          });

      Color currentColor = Color.kBlack;
      for (int led = 0; led < bufLen; led++) {
        currentColor = Objects.requireNonNullElse(stopPositions.get(led), currentColor);

        writer.setLED(led, currentColor);
      }
    };
  }

  /**
   * Creates an animation that displays a static gradient of colors across the entire length of the
   * LED strip. The gradient wraps around so the start and end of the strip are the same color,
   * which allows the gradient to be modified with a scrolling effect with no discontinuities.
   * Colors are evenly distributed along the full length of the LED strip.
   *
   * @param colors the colors to display in the gradient
   * @return the static gradient
   */
  static Animation gradient(Color... colors) {
    if (colors.length == 0) {
      // Nothing to display
      DriverStation.reportWarning("Creating a gradient with no colors!", false);
      return OFF;
    }

    if (colors.length == 1) {
      // No gradients with one color
      DriverStation.reportWarning("Creating a gradient with only one color!", false);
      return solid(colors[0]);
    }

    final int numSegments = colors.length;

    return (reader, writer) -> {
      int bufLen = reader.getLength();
      int ledsPerSegment = bufLen / numSegments;

      for (int led = 0; led < bufLen; led++) {
        int colorIndex = (led / ledsPerSegment) % numSegments;
        int nextColorIndex = (colorIndex + 1) % numSegments;
        double t = (led / (double) ledsPerSegment) % 1;

        // TODO: Avoid allocations here
        writer.setLED(led, lerp(colors[colorIndex], colors[nextColorIndex], t));
      }
    };
  }

  /**
   * Linearly interpolates between color A and B in the RGB space.
   *
   * <p><strong>TODO: interpolate in a more human-friendly colorspace like HSV or LAB.</strong>
   *
   * @param a the first color
   * @param b the second color
   * @param t how far to interpolate, as a value between 0 and 1
   * @return the interpolated color value
   */
  static Color lerp(Color a, Color b, double t) {
    return new Color(
        MathUtil.interpolate(a.red, b.red, t),
        MathUtil.interpolate(a.green, b.green, t),
        MathUtil.interpolate(a.blue, b.blue, t));
  }

  /**
   * Linearly interpolates between color A and B in the RGB space.
   *
   * <p><strong>TODO: interpolate in a more human-friendly colorspace like HSV or LAB.</strong>
   *
   * @param a the first color
   * @param b the second color
   * @param t how far to interpolate, as a value between 0 and 1
   * @return the interpolated color value
   */
  static Color8Bit lerp(Color8Bit a, Color8Bit b, double t) {
    return new Color8Bit(
        (int) Math.round(MathUtil.interpolate(a.red, b.red, t)),
        (int) Math.round(MathUtil.interpolate(a.green, b.green, t)),
        (int) Math.round(MathUtil.interpolate(a.blue, b.blue, t)));
  }
}
