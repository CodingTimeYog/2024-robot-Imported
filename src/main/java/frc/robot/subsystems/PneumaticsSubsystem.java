package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {

  private final Compressor compressor;
  private final DoubleSolenoid doubleSolenoid;
  private DoubleSolenoid.Value pistonDirection = DoubleSolenoid.Value.kOff;

  public PneumaticsSubsystem() {

    compressor = new Compressor(0, Constants.PCM_TYPE);
    System.out.println("compressor: " + compressor); // !!!SID!!! debug

    doubleSolenoid =
        new DoubleSolenoid(
            Constants.PCM_TYPE,
            Constants.LIFTER_SOLENOID_A_VALVE_ID,
            Constants.LIFTER_SOLENOID_B_VALVE_ID);
    System.out.println("solenoid: " + doubleSolenoid);

    // Start the compressor manually
    // compressor.start(); // !!!SID!!! not needed!!!
  }

  // Method to extend the piston
  public void extendPiston() {
    System.out.println("extendPiston"); // !!!SID!!! debug
    pistonDirection = DoubleSolenoid.Value.kForward;
    // doubleSolenoid.set(pistonDirection);
  }

  // Method to retract the piston
  public void retractPiston() {
    System.out.println("retractPiston"); // !!!SID!!! debug
    pistonDirection = DoubleSolenoid.Value.kReverse;
    // doubleSolenoid.set(pistonDirection);
  }

  // Method to retract the piston
  public void stopPiston() {
    pistonDirection = DoubleSolenoid.Value.kOff;
    // doubleSolenoid.set(pistonDirection);
  }

  // Method to get the current state of the piston
  public DoubleSolenoid.Value getPistonState() {
    return doubleSolenoid.get();
  }

  // get the direction we think we're going
  public DoubleSolenoid.Value getOurDirState() {
    return pistonDirection;
  }

  // Method to check if the compressor is running
  public boolean isCompressorRunning() {
    return compressor.isEnabled();
  }

  // Method to check if the pressure switch is triggered
  public boolean isPressureSwitchTriggered() {
    return compressor.getPressureSwitchValue();
  }

  // Method to check if the piston is fully extended using a limit switch
  public boolean isPistonFullyExtended() {
    boolean ret = false;
    // ret = extendedLimitSwitch.get(); //!!!SID!!! debug
    return ret;
  }

  // Method to check if the piston is fully retracted using a limit switch
  public boolean isPistonFullyRetracted() {
    boolean ret = false;
    // ret = retractedLimitSwitch.get(); //!!!SID!!! debug
    return ret;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    doubleSolenoid.set(pistonDirection);
  }
}
