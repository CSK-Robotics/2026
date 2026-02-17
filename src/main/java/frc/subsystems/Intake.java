package frc.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    public SparkMax intakeMotor;
    public SparkMax feederMotor;

    public Intake(){
        intakeMotor = new SparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        feederMotor = new SparkMax(IntakeConstants.feederMotorID, MotorType.kBrushless);
    }

    public void runForward(){
        intakeMotor.setVoltage(6);
        feederMotor.setVoltage(8);
    }

    public void runBackwards(){
        intakeMotor.setVoltage(-3);
        feederMotor.setVoltage(-3);
    }

    public void stop(){
        intakeMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public Command load(){
        return Commands.startEnd(this::runForward, this::runBackwards, this);
    }

    public Command purge(){
        return Commands.startEnd(this::runBackwards,this::stop, this);
    }

    @Override
    public void periodic() {
    
  }
}
