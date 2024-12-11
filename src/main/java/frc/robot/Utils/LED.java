package frc.robot.Utils;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LED {

  private CANdle candle;
  private CANdleConfiguration config;
  private int r, g, b, w, startIdx, count;

  public LED() {
    candle = new CANdle(0); // creates a new CANdle with ID 0
    config = new CANdleConfiguration();

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
    candle.setLEDs(255, 255, 255, 0, 10, 100); // set the CANdle LEDs to white
    // candle.setLEDs(100, 0, 0, 0, 10, 100); // set the CANdle LEDs to red
    r = 255;
    g = 255;
    b = 255;
    w = 0;
    startIdx = 10;
    count = 100;
  }

  // Getter and setter methods for RGB values for the setLED's method
  public int getR() {
    return r;
  }

  public int getG() {
    return g;
  }

  public int getB() {
    return b;
  }

  public int getW() {
    return w;
  }

  public int getStartIdx() {
    return startIdx;
  }

  public int getCount() {
    return count;
  }

  public void setR(int r) {
    this.r = r;
  }

  public void setG(int g) {
    this.g = g;
  }

  public void setB(int b) {
    this.b = b;
  }

  public void setW(int w) {
    this.w = w;
  }

  public void setStartIdx(int startIdx) {
    this.startIdx = startIdx;
  }

  public void setCount(int count) {
    this.count = count;
  }

  public void setColor(int r, int g, int b, int w, int startIdx, int count) {
    this.r = r;
    this.g = g;
    this.w = w;
    this.startIdx = startIdx;
    this.count = count;
    setColor(r, g, b, w, startIdx, count);
  }

  // Run in teleop periodic
  // FIXME Create another class that handles beams and one more for limit switches, and use limit
  // switches to check if a note is present.
  // USe the LED class to then set the color of the LED to a cetain color in the Limit switch class
  // public boolean hasNote(int r, int g, int b, int w, int startIdx, int count) {
  //   if (lim1.get()) {
  //     this.r = r;
  //     this.g = g;
  //     this.b = b;
  //     this.w = w;
  //     this.startIdx = startIdx;
  //     this.count = count;
  //     candle.setLEDs(r, g, b, w, startIdx, count);
  //     // set the CANdle LEDs to red

  //   }
  //   return lim1.get();
  // }
}
