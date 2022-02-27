// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  //static Drivetrain m_Instance = null; // old

  private static final double kCountsPerRevolution = 1440.0;  
  private static final double kWheelDiameterInch = 2.75591; // 70 mm


  private double lastLeftDistance;
  private double lastRightDistance;
  private double lastTimePassed;


  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor/* = new Spark(0)*/;
  private final Spark m_rightMotor/* = new Spark(1)*/;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder /*= new Encoder(4, 5)*/;
  private final Encoder m_rightEncoder /*= new Encoder(6, 7)*/;

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive/* = new DifferentialDrive(m_leftMotor, m_rightMotor) */;

  // Set up the RomiGyro
  private final RomiGyro m_gyro /*= new RomiGyro()*/;

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer /*= new BuiltInAccelerometer()*/;
  
  public static Drivetrain m_Instance;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor = new Spark(0);
    m_rightMotor = new Spark(1);
    m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftEncoder = new Encoder(4, 5);
    m_rightEncoder = new Encoder(6, 7);
    m_gyro = new RomiGyro();
    m_accelerometer = new BuiltInAccelerometer();
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    lastLeftDistance = getLeftDistanceInches();
    lastRightDistance = getRightDistanceInches();
    lastTimePassed = Timer.getFPGATimestamp();
  }

  public static Drivetrain getInstance() {
    if (m_Instance == null) {
      synchronized (Drivetrain.class) {
        if (m_Instance == null) {
          m_Instance = new Drivetrain();
        }
      }
    }
    return m_Instance;
  }

  public void setLeftVolts(double a) {
    m_leftMotor.setVoltage(a);
  }

  public void setRightVolts(double a) {
    m_rightMotor.setVoltage(a);
  }

  public double getLeftMotor() {
    return m_leftMotor.get();
  }
  public double getRightMotor() {
    return m_rightMotor.get();
  }

  public void setLeftMotorLevel(double a) {
    m_leftMotor.set(a);
  }

  public void setRightMotorLevel(double a) {
    m_rightMotor.set(a);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInches() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInches() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInches() + getRightDistanceInches()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double deltaT = Timer.getFPGATimestamp() - lastTimePassed;
    SmartDashboard.putNumber("LDI-JSDT", getLeftDistanceInches());
    SmartDashboard.putNumber("RDI-JSDT", getRightDistanceInches());
    if (deltaT > 0) {
      SmartDashboard.putNumber("LVT-JSDT", Units.inchesToMeters(getLeftDistanceInches() - lastLeftDistance) / deltaT);
    }
    if (deltaT > 0) {
      SmartDashboard.putNumber("RVT-JSDT", Units.inchesToMeters(getRightDistanceInches() - lastRightDistance / deltaT));
    }
    lastLeftDistance = getLeftDistanceInches();
    lastRightDistance = getRightDistanceInches();
    lastTimePassed = Timer.getFPGATimestamp();
    // SmartDashboard.putNumber("LeftML", m_leftMotor.get());
    // SmartDashboard.putNumber("RightML", m_rightMotor.get());
  }
}
