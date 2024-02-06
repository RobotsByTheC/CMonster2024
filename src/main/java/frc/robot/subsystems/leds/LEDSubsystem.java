package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledData;
  
    // Use a view for compatibility with animations
    private final AddressableLEDBufferView dataView;
  
    public LEDSubsystem() {
      leds = new AddressableLED(Constants.LEDConstants.ledPortNumber);
      leds.setLength(Constants.LEDConstants.ledLength);
      leds.start();
      ledData = new AddressableLEDBuffer(Constants.LEDConstants.ledLength);
      dataView = new AddressableLEDBufferView(ledData, 0, ledData.getLength()-1);
      setDefaultCommand(greenPurpleScroll());
    }
  
    @Override
    public void periodic() {
      leds.setData(ledData);
    }
  
    public Command runAnimation(Animation animation) {
      return run(() -> animation.update(dataView));
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
        return runAnimation(Animation.solid(Color.kViolet).blink(Seconds.of(1)));
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

    // But we can make it move! Use the "scrollAtRelativeSpeed" method to make it scroll across the LED strip
    // This will make a full trip around every 2 seconds (50% per second)
    private static final Animation rainbowFlagScroll =
        rainbowFlag.scrollAtRelativeSpeed(Percent.per(Second).of(50));

    public Command rainbowFlagScroll() {
        return runAnimation(rainbowFlagScroll);
    }

    private static final Animation greenPurple =
        Animation.steps(
        Map.of(
            0.0 / 2, Color.kLimeGreen,
            1.0 / 4, Color.kPurple,
            1.0 / 2, Color.kLimeGreen,
            3.0 / 4, Color.kPurple));
    
    private static final Animation greenPurpleScroll = 
        greenPurple.scrollAtRelativeSpeed(Percent.per(Second).of(50));

    public Command greenPurpleScroll() {
        return runAnimation(greenPurpleScroll);
    }
  }
