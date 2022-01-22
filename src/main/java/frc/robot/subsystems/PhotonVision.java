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

public class PhotonVision extends SubsystemBase {
  NetworkTable _nt;
  PIDController _pc;

  private static PhotonVision _instance = new PhotonVision();
  private Motor _m = Motor.getInstance();
  
  // this camera sucks, use a different one for comp
  String kCameraName = "C922_Pro_Stream_Webcam";

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    _nt = NetworkTableInstance.getDefault()
      .getTable("photonvision")
      .getSubTable(kCameraName);
    
    // Values for Neo 550 on the turret
    _pc = new PIDController(0.0045,0,0);
  }

  public void putTargetValues() {
    put("Ball Yaw", entry("targetYaw").getDouble(0.));
    put("Ball Pitch", entry("targetPitch").getDouble(0.));
    put("Ball Area", entry("targetArea").getDouble(0.));
  }

  public void run() {
    double yaw = entry("targetYaw").getDouble(0.);

    // requires negative on the turret for some reason
    double speed = -_pc.calculate(yaw);
    _m.set(speed);
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

  public static PhotonVision getInstance() {
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
