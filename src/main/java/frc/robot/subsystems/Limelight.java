// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable _nt;
  PIDController _pc;

  double kTargetHeight, kMountHeight, kMountAngle;

  private Motor _m = Motor.getInstance();

  private static Limelight _instance = new Limelight();
  /** Creates a new Limelight. */
  public Limelight() {
    _nt = NetworkTableInstance.getDefault().getTable("limelight");
    set("ledMode", 0);

    /*// Values for the camera taped on the side of a table,
    // and reflective tape on Sharkbait's gear target.
    // change to real comp values
    kTargetHeight = 20.;
    kMountHeight = 27.5;
    kMountAngle = 5.;*/

    // Values for table mount + Sharkbait target
    kTargetHeight = 16;
    kMountHeight = 6.25;
    kMountAngle = 0;

    // Values for Neo 550 on the turret
    _pc = new PIDController(0.0045,0,0);
  }

  public void run() {
    _m.set(-_pc.calculate(entry("tx").getDouble(0.)));
  }

  public double distance() {
    if (entry("tv").getDouble(0) == 1.) {
      double targetAngle = entry("ty").getDouble(0.);

      // sussy
      double dist = Math.abs(
        (kMountHeight - kTargetHeight) / 
        Math.tan(Math.toRadians(kMountAngle + targetAngle)));

      SmartDashboard.putNumber("Target Distance", dist);
      return dist;
    } else {
      return 0.;
    }
  }

  public void putTargetValues() {
    put("Target X Offset", entry("tx").getDouble(0.));
    put("Target Y Offset", entry("ty").getDouble(0.));
    put("Target Area", entry("ta").getDouble(0.));
  }

  public boolean set(String key, double val) {
    return entry(key).setNumber(val);
  }

  public NetworkTableEntry entry(String key) {
    return _nt.getEntry(key);
  }

  public void put(String key, double val) {
    SmartDashboard.putNumber(key, val);
  }

  public static Limelight getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
