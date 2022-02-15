/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Motor extends SubsystemBase {
  /**
   * Creates a new motor.
   */
  private CANSparkMax _m;
  private RelativeEncoder _enc;
  private static Motor _instance = new Motor();
  
  public static Motor getInstance() {
    return _instance;
  }
  public Motor() {
    // WHO CHANGED THIS TO 1
    _m = new CANSparkMax(1, MotorType.kBrushless);
    _enc = _m.getEncoder();
    zeroEncoder();
  }

  public void zeroEncoder() {
    _enc.setPosition(0.);
  }

  public void set(double vbus) {
    _m.set(vbus);
  }

  public void stop() {
    _m.set(0.);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
