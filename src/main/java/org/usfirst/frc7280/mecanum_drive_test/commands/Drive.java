// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc7280.mecanum_drive_test.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc7280.mecanum_drive_test.Robot;

/**
 *
 */
public class Drive extends Command {

    public Drive() {
        
        requires(Robot.base);

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (Robot.oi.motionStick.getRawButton(3)){
            Robot.judge.highSpeedOn = false;
            
            switch (Robot.netWorkTable.upTape) {
                case 999: // can't find tape
                    // Robot.base.drive(Robot.oi.motionStick.getY(), Robot.oi.motionStick.getX(), Robot.oi.motionStick.getZ());
                    SmartDashboard.putNumber("test", 000);
                    break;
                    
                case 1: // need to turn right
                    // Robot.base.drive(0, 0, 0.8);
                    SmartDashboard.putNumber("test", 111);
                    break;
                
                case 0: // the angle is 0 
                    switch (Robot.netWorkTable.midPoint){
                        case -1://centre is on the left
                        // Robot.base.drive(0, 0.5, 0);
                        SmartDashboard.putNumber("test", 222);
                        break;

                        case 0: //mid piont is in the centre
                        // Robot.base.drive(-0.5, 0, 0);
                        SmartDashboard.putNumber("test", 000);
                        break;

                        case 1: //centre is on the right
                        // Robot.base.drive(0, -0.5, 0);
                        SmartDashboard.putNumber("test", 333);
                        break;

                        case 5: //default
                        // Robot.base.drive(Robot.oi.motionStick.getY(), Robot.oi.motionStick.getX(), Robot.oi.motionStick.getZ());
                        SmartDashboard.putNumber("test", 000);
                        break;
                    }
                    break;

                case -1: // the need to turn left
                    // Robot.base.drive(0, 0, -0.8);
                    SmartDashboard.putNumber("test", 444);
                    break;

                case 5: //default
                    // Robot.base.drive(Robot.oi.motionStick.getY(), Robot.oi.motionStick.getX(), Robot.oi.motionStick.getZ());
                    SmartDashboard.putNumber("test", 000);
                    break;
            }
        } else {
            Robot.base.drive(Robot.oi.motionStick.getY(), Robot.oi.motionStick.getX(), Robot.oi.motionStick.getZ());
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        // end();
        // two cases: 1 the base can move, 2 the base can't move
        Robot.base.stop();
    }
}