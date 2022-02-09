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

public class ForwardPID extends CommandBase {
  /** Creates a new ForwardPID. */
  Drivetrain m_drivetrain;
  private double m_startTime, elapsed_time, position, velocity, acceleration, m_startLeftMeters, m_startRightMeters;
  private double m_leftTravel, m_rightTravel;
  private double m_tolerance, m_distance;
  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(Constants.kMaxSpeed, Constants.kMaxAcceleration);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(Units.inchesToMeters(96), 0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
  
  public ForwardPID( double distance, double tolerance, Drivetrain m_drive ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = m_drive;
    m_tolerance = tolerance;
    m_distance = distance;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp(); // returns system clock time in seconds (double)
    m_startLeftMeters  = m_drivetrain.getLeftDistanceInch();
    m_startRightMeters = m_drivetrain.getRightDistanceInch();
    System.out.println("Initialized!");
    System.out.println(m_profile.totalTime());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // timer
    elapsed_time = Timer.getFPGATimestamp() - m_startTime;
    // real distance traveled
    m_leftTravel  = m_drivetrain.getLeftDistanceInch()  - m_startLeftMeters;
    m_rightTravel = m_drivetrain.getRightDistanceInch() - m_startRightMeters;
    System.out.println("Elapsed Time" + elapsed_time);
    // 
    double expected_distance, expected_velocity, expected_acceleration;
    if(elapsed_time > m_profile.totalTime()) {
      expected_distance = m_distance;
      expected_velocity = 0;
      expected_acceleration = 0;
      System.out.println("If ran!!");
    }
    else {
      TrapezoidProfile.State expected_state = m_profile.calculate(elapsed_time);
      TrapezoidProfile.State new_State = m_profile.calculate(elapsed_time + Constants.kSecondsPerCycle);
      expected_distance = expected_state.position;
      System.out.println("Position: " + position);
      expected_velocity = expected_state.velocity;
      System.out.println("Velocity: " + velocity);
      expected_acceleration = (new_State.velocity - expected_state.velocity)/Constants.kSecondsPerCycle;
      System.out.println("Else ran!!");
    }

    double leftError = expected_distance - m_leftTravel;
    double rightError = expected_distance - m_rightTravel;

    double left_voltage = Constants.ksVolts
                           + expected_velocity * Constants.kvVolts
                           + expected_acceleration * Constants.kaVolts
                           + leftError * Constants.kpDriveVel;

    double right_voltage = Constants.ksVolts
                           + expected_velocity * Constants.kvVolts
                           + expected_acceleration * Constants.kaVolts
                           + rightError * Constants.kpDriveVel;

    m_drivetrain.setLeftVolts(left_voltage);
    m_drivetrain.setRightVolts(right_voltage);

    SmartDashboard.putNumber("Expected Distance", position);
    SmartDashboard.putNumber("Expected Velocity", velocity);
    SmartDashboard.putNumber("Expected Acceleration", acceleration);
    SmartDashboard.putNumber("Left Travel", m_leftTravel);
    SmartDashboard.putNumber("Right Travel", m_rightTravel);
    SmartDashboard.putNumber("Left Error", leftError);
    SmartDashboard.putNumber("Right Error", rightError);
    SmartDashboard.putNumber("Left Voltage", left_voltage);
    SmartDashboard.putNumber("Right Voltage", right_voltage);
    SmartDashboard.putNumber("Left Shortage", m_leftTravel - position);
    SmartDashboard.putNumber("Right Shortage", m_rightTravel - position);
    SmartDashboard.putNumber("Elapsed Time", elapsed_time);
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftVolts(0);
    m_drivetrain.setRightVolts(0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_leftTravel  - position) < m_tolerance)
           &&
           (Math.abs(m_rightTravel - position) < m_tolerance);
  }
}
