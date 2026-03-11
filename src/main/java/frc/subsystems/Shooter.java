package frc.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class Shooter extends SubsystemBase {
  private final SparkFlex feederRoller;
  private final SparkFlex intakeLauncherRoller;
  // private final LightsSubsystem lights;
  private final RelativeEncoder shooterEncoder;

  /** Creates a new CANBallSubsystem. */
  public Shooter() { //LightsSubsystem lights
    // this.lights = lights;

    intakeLauncherRoller = new SparkFlex(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkFlex(FEEDER_MOTOR_ID, MotorType.kBrushless);

    shooterEncoder = intakeLauncherRoller.getEncoder();

    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig launcherConfig = new SparkFlexConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    System.out.println("Intake");
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
    // lights.setMode(LightsSubsystem.Mode.INTAKING);
  }

  public void eject() {
    System.out.println("Eject");
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(
        -1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
    // lights.setMode(LightsSubsystem.Mode.ERROR);
  }

  public void launch() {
    System.out.println("Launch");
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(
        SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    // lights.setMode(LightsSubsystem.Mode.LAUNCHING);
  }

  public void stop() {
    System.out.println("Stop");
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    // lights.setMode(LightsSubsystem.Mode.IDLE);
  }

  public void spinUp() {
    System.out.println("Spin Up");
    feederRoller.setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(
        SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    // lights.setMode(LightsSubsystem.Mode.SPINUP);
  }

  public double getShooterRPM() {
    return shooterEncoder.getVelocity();
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command shootingCommand() {
    return this.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(this.launchCommand()).finallyDo(() -> this.stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
  }
}