package frc.robot.subsystems.leds;

import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kPurple;
import static edu.wpi.first.wpilibj.util.Color.kWhite;
import static edu.wpi.first.wpilibj.util.Color.kYellow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Map;
import org.junit.jupiter.api.Test;

class AnimationTest {
  @Test
  void solidColor() {
    Animation animation = Animation.solid(kYellow);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);

    for (int i = 0; i < buffer.getLength(); i++) {
      assertEquals(kYellow, buffer.getLED(i));
    }
  }

  @Test
  void gradient0SetsToBlack() {
    Animation animation = Animation.gradient();
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 127, 128, 129);
    }

    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);

    for (int i = 0; i < buffer.getLength(); i++) {
      assertEquals(kBlack, buffer.getLED(i));
    }
  }

  @Test
  void gradient1SetsToSolid() {
    Animation animation = Animation.gradient(kYellow);

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);

    for (int i = 0; i < buffer.getLength(); i++) {
      assertEquals(kYellow, buffer.getLED(i));
    }
  }

  @Test
  void gradient2Colors() {
    Animation animation = Animation.gradient(kYellow, kPurple);

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);

    assertColorEquals(kYellow, buffer.getLED(0));
    assertColorEquals(Animation.lerp(kYellow, kPurple, 25 / 49.0), buffer.getLED(25));
    assertColorEquals(kPurple, buffer.getLED(49));
    assertColorEquals(Animation.lerp(kYellow, kPurple, 25 / 49.0), buffer.getLED(73));
    assertColorEquals(kYellow, buffer.getLED(98));
  }

  @Test
  void gradient3Colors() {
    Animation animation = Animation.gradient(kYellow, kPurple, kWhite);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);

    assertColorEquals(kYellow, buffer.getLED(0));
    assertColorEquals(Animation.lerp(kYellow, kPurple, 25.0 / 33.0), buffer.getLED(25));
    assertColorEquals(kPurple, buffer.getLED(33));
    assertColorEquals(Animation.lerp(kPurple, kWhite, 25.0 / 33.0), buffer.getLED(58));
    assertColorEquals(kWhite, buffer.getLED(66));
    assertColorEquals(Animation.lerp(kWhite, kYellow, 25.0 / 33.0), buffer.getLED(91));
    assertColorEquals(Animation.lerp(kWhite, kYellow, 32.0 / 33.0), buffer.getLED(98));
  }

  @Test
  void step0SetsToBlack() {
    Animation animation = Animation.steps(Map.of());

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 127, 128, 129);
    }

    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);
    for (int i = 0; i < 99; i++) {
      assertColorEquals(kBlack, buffer.getLED(i));
    }
  }

  @Test
  void step1SetsToSolid() {
    Animation animation = Animation.steps(Map.of(0.0, kYellow));

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);

    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);
    for (int i = 0; i < 99; i++) {
      assertColorEquals(kYellow, buffer.getLED(i));
    }
  }

  @Test
  void step1HalfSetsToHalfOffHalfColor() {
    Animation animation = Animation.steps(Map.of(0.50, kYellow));

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(99);
    animation.update(LEDReader.forBuffer(buffer), buffer::setRGB);

    // [0, 48] should be black...
    for (int i = 0; i < 49; i++) {
      assertColorEquals(kBlack, buffer.getLED(i));
    }
    // ... and [49, <end>] should be the color that was set
    for (int i = 49; i < buffer.getLength(); i++) {
      assertColorEquals(kYellow, buffer.getLED(i));
    }
  }

  void assertColorEquals(Color expected, Color actual) {
    assertEquals(new Color8Bit(expected), new Color8Bit(actual));
  }
}
