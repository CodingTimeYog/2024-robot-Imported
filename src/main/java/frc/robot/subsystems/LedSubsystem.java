package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
  private final CANdle mCandle = new CANdle(Constants.LED_STRIP_CAN_ID);

  public LedSubsystem() {
    var candleConfig = new CANdleConfiguration();
    candleConfig.stripType = CANdle.LEDStripType.GRB;
    candleConfig.v5Enabled = false;
    candleConfig.disableWhenLOS = false;
    candleConfig.enableOptimizations = true;
    mCandle.configAllSettings(candleConfig);

    mCandle.setLEDs(0, 0, 0, 0, 0, 8);
    mCandle.clearAnimation(0);
    setDefaultCommand(writeStaticColor(255, 0, 0, 0.5));
  }

  private void resetAnimation() {
    mCandle.clearAnimation(0);
  }

  public Command writeStaticColor(int pRed, int pGreen, int pBlue, double pBrightness) {
    return makeAnimationCommand(
        new SingleFadeAnimation(pRed, pGreen, pBlue, 255, 0.5, Constants.LED_STRIP_NUM_LEDS, 8));
  }

  public Command strobeColor(int pRed, int pGreen, int pBlue) {
    return this.runEnd(
            () -> mCandle.animate(new StrobeAnimation(pRed, pGreen, pBlue, 255, 0.25, 512, 8)),
            this::resetAnimation)
        .ignoringDisable(true);
  }

  public Command rainbow(double pBrightness) {
    return this.runEnd(
            () ->
                mCandle.animate(
                    new RainbowAnimation(
                        pBrightness, 0.25, Constants.LED_STRIP_NUM_LEDS, false, 8)),
            this::resetAnimation)
        .ignoringDisable(true);
  }

  public Command colorFadeCommand(int pRed, int pGreen, int pBlue) {
    return this.runEnd(
        () ->
            mCandle.animate(
                new ColorFlowAnimation(
                    pRed,
                    pGreen,
                    pBlue,
                    255,
                    0.25,
                    Constants.LED_STRIP_NUM_LEDS,
                    Direction.Backward,
                    8)),
        this::resetAnimation);
  }

  private Command makeAnimationCommand(Animation pAnimation) {
    return this.runEnd(() -> mCandle.animate(pAnimation), this::resetAnimation)
        .ignoringDisable(true);
  }
}
