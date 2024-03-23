package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledData;

  // Use a view for compatibility with all
  private final AddressableLEDBufferView all;
  private final AddressableLEDBufferView rightUpperDiagonal;
  private final AddressableLEDBufferView rightLowerDiagonal;
  private final AddressableLEDBufferView rightVertical;
  private final AddressableLEDBufferView leftUpperDiagonal;
  private final AddressableLEDBufferView leftLowerDiagonal;
  private final AddressableLEDBufferView leftVertical;
  private final AddressableLEDBufferView across;

  public LEDSubsystem() {
    leds = new AddressableLED(Constants.LEDConstants.ledPortNumber);
    leds.setLength(Constants.LEDConstants.ledLength);
    leds.start();
    ledData = new AddressableLEDBuffer(Constants.LEDConstants.ledLength);
    all = new AddressableLEDBufferView(ledData, 0, ledData.getLength() - 1);
    setDefaultCommand(greenPurpleScroll());
    rightUpperDiagonal = new AddressableLEDBufferView(ledData, 88, 110).reversed();
    rightLowerDiagonal = new AddressableLEDBufferView(ledData, 111, 132);
    rightVertical = new AddressableLEDBufferView(ledData, 133, 149).reversed();
    leftUpperDiagonal = new AddressableLEDBufferView(ledData, 43, 64);
    leftLowerDiagonal = new AddressableLEDBufferView(ledData, 20, 42).reversed();
    leftVertical = new AddressableLEDBufferView(ledData, 0, 19);
    across = new AddressableLEDBufferView(ledData, 65, 87);
  }

  @Override
  public void periodic() {
    leds.setData(ledData);
  }

  public Command runAnimation(Animation animation) {
    return run(() -> animation.update(all)).ignoringDisable(true);
  }

  public Command runSpecialAnimation(Animation animation) {
    return run(() -> {
          animation.update(rightUpperDiagonal);
          animation.update(rightLowerDiagonal);
          animation.update(rightVertical);
          animation.update(leftUpperDiagonal);
          animation.update(leftLowerDiagonal);
          animation.update(leftVertical);
          animation.update(across);
        })
        .ignoringDisable(true);
  }

  public Command blinkRed() {
    return runAnimation(Animation.solid(Color.kRed).blink(Seconds.of(1)));
  }

  public Command blinkBlue() {
    return runAnimation(Animation.solid(Color.kBlue).blink(Seconds.of(1)));
  }

  public Command blinkGreen() {
    return runAnimation(Animation.solid(Color.kGreen).blink(Seconds.of(1)));
  }

  public Command blinkPurple() {
    return runAnimation(Animation.solid(Color.kPurple).blink(Seconds.of(1)));
  }

  public Command blinkYellow() {
    return runAnimation(Animation.solid(Color.kOrange).blink(Seconds.of(1)));
  }

  public Command greenPurpleGradient() {
    return runSpecialAnimation(
        Animation.gradient(Color.kGreen, Color.kPurple)
            .scrollAtRelativeSpeed(Percent.per(Second).of(30)));
  }

  private static final Animation rainbowFlag =
      Animation.steps(
          Map.of(
              0.0 / 6, Color.kRed,
              1.0 / 6, Color.kOrange,
              2.0 / 6, Color.kYellow,
              3.0 / 6, Color.kGreen,
              4.0 / 6, Color.kBlue,
              5.0 / 6, Color.kViolet));

  // But we can make it move! Use the "scrollAtRelativeSpeed" method to make it scroll across the
  // LED strip
  // This will make a full trip around every 2 seconds (50% per second)
  private static final Animation rainbowFlagScroll =
      rainbowFlag.scrollAtRelativeSpeed(Percent.per(Second).of(50));

  public Command rainbowFlagScroll() {
    return runSpecialAnimation(rainbowFlagScroll);
  }

  private static final Animation greenPurple =
      Animation.steps(
          Map.of(
              0.0 / 2, Color.kGreen,
              1.0 / 4, Color.kPurple,
              1.0 / 2, Color.kGreen,
              3.0 / 4, Color.kPurple));

  private static final Animation redBlue =
      Animation.steps(
          Map.of(
              0.0 / 2, Color.kRed,
              1.0 / 4, Color.kBlue));

  private static final Animation greenPurpleScroll =
      greenPurple.scrollAtRelativeSpeed(Percent.per(Second).of(50));

  private static final Animation redBlueScroll =
      redBlue.scrollAtRelativeSpeed(Percent.per(Second).of(50));

  public Command greenPurpleScroll() {
    return runAnimation(greenPurpleScroll);
  }

  public Command crazyWhiteBlink() {
    return runAnimation(Animation.solid(Color.kWhite).blink(Seconds.of(.1)));
  }

  public Command police() {
    return runSpecialAnimation(redBlueScroll);
  }
}
