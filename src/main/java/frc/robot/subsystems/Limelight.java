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

  double kTargetHeight, kMountHeight, kMountAngle, kDistIters;

  private int distEstIters = 0;
  private double distEst, distEstTotal;

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
    kMountHeight = 6;
    kMountAngle = -0.45;
    // These values produce high-accuracy results at 90-120in,
    // with a Y offset of ~6. At the competition, we're going
    // to need to find a good distance for high-accuracy.
    // I'm thinking 8-12 feet should probably be good;
    // it's where Auton will shoot from, and the only area
    // that will have a large range of required powers, and maybe angle settings
    
    // It'll probably usually be setting 3, but it's not as clear-cut
    // as close and far shots (1 and 3 resp.). plus, power needs to be adjusted.
    // In the end, we're probably going to have manual adjustments ready,
    // in case Limelight dies.

    kDistIters = 20;

    // Limelight is currently set to work on dual target
    // I think the competition goal is tri target

    // Values for Neo 550 on the turret
    _pc = new PIDController(0.0045,0,0);

    put("Mount Angle", kMountAngle);
    put("Mount Height", kMountHeight);
    put("Target Height", kTargetHeight);
    put("Distance Iterations", kDistIters);
  }

  public void run() {
    _m.set(-_pc.calculate(entry("tx").getDouble(0.)));
  }

  public double distance() {
    if (entry("tv").getDouble(0) == 1.) {
      //System.out.println(distEstIters);
      //System.out.println(kDistIters);
      if (distEstIters >= kDistIters) {
        distEst = distEstTotal / distEstIters;
        distEstTotal = 0;
        distEstIters = 0;
        put("Target Distance", distEst);
      }

      double targetAngle = entry("ty").getDouble(0.);

      // This is extremely sussy but it works perfectly so I'm going to keep it
      double dist = Math.abs(
        (kMountHeight - kTargetHeight) / 
        Math.tan(Math.toRadians(kMountAngle + targetAngle)));
      
      System.out.println(dist);
      distEstTotal += dist;
      distEstIters++;
      put("Distance Estimation Total", dist);
    }

    return distEst;
  }

  public void update() {
    double mountAngle = get("Mount Angle", 0.);
    double mountHeight = get("Mount Height", 0.);
    double targetHeight = get("Target Height", 0.);
    double distIters = get("Distance Iterations", 0.);

    if (mountAngle != kMountAngle) { kMountAngle = mountAngle; }
    if (mountHeight != kMountHeight) { kMountHeight = mountHeight; }
    if (targetHeight != kTargetHeight) { kTargetHeight = targetHeight; }
    if (distIters != kDistIters) { kDistIters = distIters; }
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

  public double get(String key, double defaultValue) {
    return SmartDashboard.getNumber(key, defaultValue);
  }

  public static Limelight getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
