// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  AHRS gyro = new AHRS(SerialPort.Port.kUSB );
  static final double maxderece = 11;
  double rollAngleDegrees     = gyro.getRoll();
  

  Encoder encoderizquierda = new Encoder(0,1);
  Encoder encoderderecha = new Encoder(2,3);

  
  private final VictorSP rightMotoruno = new VictorSP(0);
  private final VictorSP rightMotordos = new VictorSP(1);
  
  //left motors

  private final VictorSP leftMotoruno = new VictorSP(2);
  private final VictorSP leftMotordos = new VictorSP(3);
   
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightMotoruno, rightMotordos);
  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftMotoruno, leftMotordos);

  //drivetrain
  
  private final DifferentialDrive  drivetrain = new DifferentialDrive(rightMotorGroup, leftMotorGroup);
  public static final double kArmOffsetRads = 0.5;
  public static final double kSVolts = 1;
  public static final double kGVolts = 1;
  public static final double kVVoltSecondPerRad = 0.5;
  public static final double kAVoltSecondSquaredPerRad = 0.1;


  public static final double kMaxAccelerationRadPerSecSquared = 10;
  public static final int kEncoderPPR = 2048;
      public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
  public static final double kMaxVelocityRadPerSecond = 3;
  private final VictorSP m_motor = new VictorSP(5);
  
  private final Encoder m_encoder = new Encoder(4, 5);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          kSVolts, kGVolts,
          kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

          public void useOutput(double output, TrapezoidProfile.State setpoint) {
            // Calculate the feedforward from the sepoint
            double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
            // Add the feedforward to the PID output to get the motor output
            m_motor.setVoltage(output + feedforward);
          }

public double getMeasurement() {
  return m_encoder.getDistance() + kArmOffsetRads;
  }
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  Joystick Joy = new Joystick(0);
  XboxController Xbox = new XboxController(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_encoder.setDistancePerPulse(kEncoderDistancePerPulse);
    setGoal(kArmOffsetRads);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  private void setGoal(double karmoffsetrads2) {
  }



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.arcadeDrive(Joy.getX(),-Joy.getY());
    if (Xbox.getAButtonPressed()) {
     setGoal(1.0472);
    
    }
    if (Xbox.getBButtonPressed()) {
      setGoal(kArmOffsetRads);
     
     }
  }
    

 @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
