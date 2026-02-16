// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.subsystems.Shooter;
import static frc.robot.Constants.FuelConstants.*;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController m_controller2 = new CommandXboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final EventLoop m_loop = new EventLoop(); 
  private final Shooter ballSubsystem = new Shooter();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private Command m_autonomousCommand;

  public boolean prevX = false;

 @Override
  public void autonomousInit() {
    m_swerve.setupAutonomousConfigure();
    m_autonomousCommand = getAutonomousCommand();
  }

  @Override
  public void autonomousPeriodic() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    m_controller.a(m_loop).ifHigh(m_swerve::reset);
  }

  @Override
  public void teleopPeriodic() {
    if(!m_controller.getXButton())driveWithJoystick(true);

    if( m_controller.getXButton()){
      m_swerve.lockWheels();
    }

    intake();
    m_loop.poll();
  }

  @Override
  public void robotPeriodic() {
    m_swerve.updateOdometry();
    CommandScheduler.getInstance().run();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = Math.pow(-m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02)), 3)
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = Math.pow(-m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02)), 3)
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = Math.pow(-m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02)), 3)
        * Drivetrain.kMaxAngularSpeed;
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }

  private void intake(){
    // While the left bumper on operator controller is held, intake Fuel
    m_controller2.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    m_controller2.rightBumper()
        .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    m_controller2.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
  }

  public Command getAutonomousCommand() {
    /*
     * try {
     * // Load the path you want to follow using its name in the GUI
     * PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath1");
     * // Create a path following command using AutoBuilder. This will also trigger
     * // event markers.
     * return AutoBuilder.followPath(path);
     * } catch (Exception e) {
     * DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
     * return Commands.none();
     * }
     */

    return m_swerve.run(() -> m_swerve.drive(0.0, 0.5, 0.0, false, getPeriod())).until(() -> isTeleop());
  }
}
