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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.Robot;
import org.usfirst.frc7280.mecanum_drive_test.RobotMap;
import org.usfirst.frc7280.mecanum_drive_test.commands.ManualElevator;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX elevatorMaster = new TalonSRX(RobotMap.elevatorMasterMotor);
  private VictorSPX elevatorSlave = new VictorSPX(RobotMap.elevatorSlaveMotor);

  RobotMap robotMap = new RobotMap();
  public int elevatorPosition;
  public int targetPosition;

  

  public Elevator(){
    
    robotMap.TalonSRXInit(elevatorMaster, Constants.kElevatorPeakOutput); 
    
    elevatorMaster.setNeutralMode(NeutralMode.Brake);
    elevatorSlave.setNeutralMode(NeutralMode.Brake);
    
    elevatorSlave.follow(elevatorMaster); 

    // set whether you need to invert the motor to get right value
    elevatorMaster.setInverted(Constants.kMotorInverted);
    elevatorSlave.setInverted(true); // modified
 
    robotMap.setMotorPID(
    elevatorMaster, 
    Constants.kElevatorF, 
    Constants.kElevatorP, 
    Constants.kElevatorI, 
    Constants.kElevatorD);

    // elevatorMaster.setSelectedSensorPosition(0, Constants.kSlotIdx, Constants.kTimeoutMs);

    // current limit 
    elevatorMaster.enableCurrentLimit(true);
    elevatorMaster.configContinuousCurrentLimit(18, Constants.kTimeoutMs);
    elevatorMaster.configPeakCurrentLimit(Constants.kPeakCurrentLimit, Constants.kTimeoutMs);
    elevatorMaster.configPeakCurrentDuration(Constants.kpeakCurrentDuration, Constants.kTimeoutMs);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ManualElevator());
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("elevator pos", elevatorMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("current _elevator", elevatorMaster.getOutputCurrent());
    SmartDashboard.putNumber("cu elevator speed", elevatorMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("cu elevator percent", elevatorMaster.getMotorOutputPercent());
  }

  public void liftToPosition(int _position){

    /* 
    evaluate PID and peak output, separate into three part
    1. elevator going up within the first frame
    2. elevator going up to using the second lift 
    3. elevator going down
    */
    targetPosition = _position;
    elevatorPosition = elevatorMaster.getSelectedSensorPosition(Constants.kSlotIdx);
    if (targetPosition < -17000 && Robot.arm.lifted == false){
      Robot.arm.lift();
    } else {
      if ( elevatorPosition < _position - 5000) { //lift up
        if (elevatorPosition < Constants.midPause ){  //lift third part 
          robotMap.setMotorPID(elevatorMaster, 0, 0.12, 0, 0);
          elevatorMaster.set(ControlMode.Velocity, 8000);
        } else {
          robotMap.setMotorPID(elevatorMaster, 0, 0.18, 0, 0);
          elevatorMaster.set(ControlMode.Velocity, 8000);
        } 
      }  else if (elevatorPosition > _position + 5000) { //down
        robotMap.setMotorPID(elevatorMaster, 0, 0.1, 0, 0);
        elevatorMaster.set(ControlMode.Velocity, -4500);
      }  
      else if (targetPosition -5000 < elevatorPosition && elevatorPosition < targetPosition - 500 ){
        robotMap.setMotorPID(elevatorMaster, 0, 0.1, 0, 0);
        elevatorMaster.set(ControlMode.Velocity, 1000);
      } else if (targetPosition +5000 > elevatorPosition && elevatorPosition > targetPosition + 500){
        robotMap.setMotorPID(elevatorMaster, 0, 0.1, 0, 0);
        elevatorMaster.set(ControlMode.Velocity, -1000);
      }
      else {
        robotMap.setMotorPID(elevatorMaster, 0, 0.01, 0, 0);
        elevatorMaster.set(ControlMode.Position, _position);
      }
    }

    // if( Robot.judge.hasBall == true){
    //   Robot.intaker.take(0.3);
    // }

    SmartDashboard.putNumber("current position", elevatorMaster.getSelectedSensorPosition(Constants.kSlotIdx));
    SmartDashboard.putNumber("Target position", _position);
    SmartDashboard.putNumber("elevator output", elevatorMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("elevator current", elevatorMaster.getOutputCurrent());
    SmartDashboard.putNumber("elevator current", elevatorMaster.getOutputCurrent());

  }

  public void elevatorDown(){
    targetPosition = elevatorPosition + 5000;
    robotMap.setMotorPID(
        elevatorMaster, 
        Constants.kElevatorDownF, 
        Constants.kElevatorDownP, 
        Constants.kElevatorDownI, 
        Constants.kElevatorDownD);
        elevatorMaster.configClosedLoopPeakOutput(Constants.kSlotIdx, Constants.kElevatorDownPeakOutput, Constants.kTimeoutMs);

    elevatorMaster.set(ControlMode.Position, targetPosition);

  }

  public void manualRun(double _outPut){
    elevatorMaster.set(ControlMode.PercentOutput, _outPut);

    SmartDashboard.putNumber("elevator position", elevatorMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("elevator current", elevatorMaster.getOutputCurrent());
    SmartDashboard.putNumber("elevator percent", elevatorMaster.getMotorOutputPercent());

  }

  public void stop(){
    elevatorMaster.set(ControlMode.PercentOutput, 0);
  }


  // use for testing
  public void testRun(double _speed){
    elevatorMaster.set(ControlMode.PercentOutput, _speed/3);
    elevatorSlave.follow(elevatorMaster);
  }

  public void test(){
    elevatorMaster.configClosedLoopPeakOutput(Constants.kSlotIdx, 0.9);
    switch (Robot.oi.motionStick.getPOV()){
      case 0:

      if (elevatorMaster.getSelectedSensorPosition() > Constants.midPause){
        robotMap.setMotorPID(elevatorMaster, 0, 0.18, 0, 0);
        elevatorMaster.set(ControlMode.Velocity, 8000);
      } else {
        robotMap.setMotorPID(elevatorMaster, 0, 0.12, 0, 0);
        elevatorMaster.set(ControlMode.Velocity, 8000);
      }
      
      break;

      case 180:

        robotMap.setMotorPID(elevatorMaster, 0, 0.15, 0, 0);
        elevatorMaster.set(ControlMode.Velocity, -5000);
        break;

      default:
      elevatorMaster.set(ControlMode.PercentOutput, 0);
      break;
      }

      
    
    }
  
    }
  


