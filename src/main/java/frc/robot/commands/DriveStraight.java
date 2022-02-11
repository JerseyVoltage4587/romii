// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final double m_distance, m_tolerance;
  private final TrapezoidProfile m_profile;
  private double m_startTime, m_startLeftMeters, m_startRightMeters;
  private double m_leftTravel, m_rightTravel;

public DriveStraight( double distance, double tolerance, Drivetrain drivetrain) {
    m_distance   = Units.feetToMeters(distance);    // meters
    m_tolerance  = Units.inchesToMeters(tolerance);   // meters
    m_drivetrain = drivetrain;
    m_profile = new TrapezoidProfile (
                    new TrapezoidProfile.Constraints(Constants.kMaxSpeed,
                                                     Constants.kMaxAcceleration),
                    new TrapezoidProfile.State(m_distance,0),
                    new TrapezoidProfile.State(0,0)
                );
    addRequirements(m_drivetrain);
    System.out.println("m_distance="+m_distance+",m_tolerance="+m_tolerance+",m_profile.totalTime="+m_profile.totalTime());
  }

  @Override
  public void initialize() {
    m_startTime        = Timer.getFPGATimestamp();
    m_startLeftMeters  = m_drivetrain.getLeftDistanceInches();
    m_startRightMeters = m_drivetrain.getRightDistanceInches();
    System.out.println("m_startTime="+m_startTime+",m_startLeft="+m_startLeftMeters+",m_startRight="+m_startRightMeters);
  }

  @Override
  public void execute() {
    double elapsed_time = Timer.getFPGATimestamp() - m_startTime;
    m_leftTravel  = m_drivetrain.getLeftDistanceInches()  - m_startLeftMeters;
    m_rightTravel = m_drivetrain.getRightDistanceInches() - m_startRightMeters;

    double expected_distance, expected_velocity, expected_acceleration;
    if ( elapsed_time > m_profile.totalTime()) {
        expected_distance     = m_distance;
        expected_velocity     = 0;
        expected_acceleration = 0;
    } else {
        TrapezoidProfile.State expected_state = m_profile.calculate(elapsed_time);
        TrapezoidProfile.State next_state     = m_profile.calculate(elapsed_time + Constants.kSecondsPerCycle);
        expected_distance     = (expected_state.position);
        expected_velocity     = expected_state.velocity;
        expected_acceleration = (next_state.velocity - expected_state.velocity) / Constants.kSecondsPerCycle;
    }

    double left_error  = expected_distance - m_leftTravel;
    double right_error = expected_distance - m_rightTravel;

    double left_voltage = Constants.ksVolts
                          + expected_velocity * Constants.kvVolts
                          + expected_acceleration * Constants.kaVolts
                          + left_error * Constants.kpDriveVel;

    double right_voltage = Constants.ksVolts
                           + expected_velocity * Constants.kvVolts
                           + expected_acceleration * Constants.kaVolts
                           + right_error * Constants.kpDriveVel;

    m_drivetrain.setLeftVolts  ( left_voltage  );
    m_drivetrain.setRightVolts ( right_voltage );

    SmartDashboard.putNumber("Expected Distance", expected_distance);
    SmartDashboard.putNumber("Expected Velocity", expected_velocity);
    SmartDashboard.putNumber("Expected Acceleration", expected_acceleration);
    SmartDashboard.putNumber("Left Travel", m_leftTravel);
    SmartDashboard.putNumber("Right Travel", m_rightTravel);
    SmartDashboard.putNumber("Left Error", left_error);
    SmartDashboard.putNumber("Right Error", right_error);
    SmartDashboard.putNumber("Left Voltage", left_voltage);
    SmartDashboard.putNumber("Right Voltage", right_voltage);
    SmartDashboard.putNumber("Left Shortage", m_leftTravel - m_distance);
    SmartDashboard.putNumber("Right Shortage", m_rightTravel - m_distance);
    SmartDashboard.putNumber("Elapsed Time", elapsed_time);
  }

  @Override
  public void end(boolean interrupted) {

        // Let the drivetrain revert to its default command.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_leftTravel  - m_distance) < m_tolerance)
           &&
           (Math.abs(m_rightTravel - m_distance) < m_tolerance);
  }
}
