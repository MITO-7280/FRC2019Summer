/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.Robot;
import org.usfirst.frc7280.mecanum_drive_test.RobotMap;
import org.usfirst.frc7280.mecanum_drive_test.commands.ManualArm;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX armMotor = new TalonSRX(RobotMap.armMotor);
  private RobotMap robotMap = new RobotMap();

  public int armPosition;
  public double armSpeed;
  public boolean lifted = false;

  public Arm() {

    armMotor.configFactoryDefault();
    robotMap.TalonSRXInit(armMotor, Constants.kArmPeakOutput);
    armMotor.setSensorPhase(false);

    armMotor.setInverted(true);

    armMotor.setNeutralMode(NeutralMode.Brake);
    robotMap.setMotorPID(armMotor, 0.09, 0, 0, 0);

    armMotor.enableCurrentLimit(true);
    armMotor.configContinuousCurrentLimit(40, Constants.kTimeoutMs);
    armMotor.configPeakCurrentLimit(50, Constants.kTimeoutMs);
    armMotor.configPeakCurrentDuration(Constants.kpeakCurrentDuration, Constants.kTimeoutMs);

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualArm());
    // modify needed check whether they will interrupt themselves
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current _arm", armMotor.getOutputCurrent());
    SmartDashboard.putBoolean("lifted", lifted);
    SmartDashboard.putNumber("arm pos", armMotor.getSelectedSensorPosition());
    armPosition = armMotor.getSelectedSensorPosition();
  }

  public void lift() {
    armMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, 1, Constants.kTimeoutMs);
    // switch(Math.abs(armMotor.getSelectedSensorPosition())){
    // case > (Constants.kLift + 800):
    // armMotor.set(ControlMode.Position, Constants.kLift);

    // }
    if (lifted) {
      robotMap.setMotorPID(armMotor, 0, 0.04, 0, 0);
      armMotor.set(ControlMode.Position, Constants.kLift+1500);
    } else {
      if (Math.abs(armMotor.getSelectedSensorPosition()) > (Constants.kLift)) {
        lifted = true;
      } else if (Math.abs(armMotor.getSelectedSensorPosition()) > (Constants.kLift - 3000)) {
        robotMap.setMotorPID(armMotor, 0, 0.5, 0, 0);
        armMotor.set(ControlMode.Velocity, 100);
      } else if (Math.abs(armMotor.getSelectedSensorPosition()) > 28000) {
        robotMap.setMotorPID(armMotor, 0, 0.3, 0, 0);
        armMotor.set(ControlMode.Velocity, 4000);
      } else {
        robotMap.setMotorPID(armMotor, 0, 0.2, 0, 0);
        armMotor.set(ControlMode.Velocity, 40000);
      }
    }

    // if (Robot.judge.hasBall) {
    //   Robot.intaker.take(0.3);
    // }

    armPosition = armMotor.getSelectedSensorPosition();
    armSpeed = armMotor.getSelectedSensorVelocity();

    SmartDashboard.putNumber("arm position", armPosition);
    SmartDashboard.putNumber("arm percetn", armMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("current _arm", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("arm target", Constants.kLift);
    SmartDashboard.putNumber("arm Speed", armSpeed);
    SmartDashboard.putNumber("arm distance", Math.abs(Robot.arm.armPosition - Constants.kLift));
  }

  public void down() {
    armMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, 0.45, Constants.kTimeoutMs);

    if (lifted) {
      if (Math.abs(armMotor.getSelectedSensorPosition()) > 28000) {
        robotMap.setMotorPID(armMotor, 0, 0.2, 0, 0);
        armMotor.set(ControlMode.Velocity, -3000);
      } else if (armMotor.getSelectedSensorPosition() > 2000) {
        robotMap.setMotorPID(armMotor, 0, 0.02, 0, 0);
        armMotor.set(ControlMode.PercentOutput, 0);
      } else if (armMotor.getSelectedSensorPosition() > 700) {
        robotMap.setMotorPID(armMotor, 0, 0.2, 0, 0);
        armMotor.set(ControlMode.Velocity, -100);
      } else {
        lifted = false;
      }
    } else {
      robotMap.setMotorPID(armMotor, 0, 0.3, 0, 0);
      armMotor.set(ControlMode.Position, 0);
    }

    armPosition = armMotor.getSelectedSensorPosition();
    armSpeed = armMotor.getSelectedSensorVelocity();

    SmartDashboard.putNumber("arm position", armPosition);
    SmartDashboard.putNumber("current _arm", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("arm target", Constants.kDown);

  }

  // manual control method for arms, control it with right stick
  public void ManualRun(double _outPut) {
    armMotor.set(ControlMode.PercentOutput, _outPut);
    armPosition = armMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("arm position", armPosition);
    SmartDashboard.putNumber("arm current", armMotor.getOutputCurrent());
  }

  public void stop() {
    armMotor.set(ControlMode.PercentOutput, 0);

  }

  public void armTest() {
    armMotor.set(ControlMode.PercentOutput, -0.8);
  }

}
