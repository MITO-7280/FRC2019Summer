
package org.usfirst.frc7280.mecanum_drive_test.command_group;

import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.Robot;
import org.usfirst.frc7280.mecanum_drive_test.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LongVisionMotion extends CommandGroup{

    public LongVisionMotion(){
        /*
        1. make the middle in centre (use another command called visionMotion)
        2. turn right or left
        3. mover forward to the vertical line of the centre of rocket/cargo
        4. make the middle in the centre (turn left/right)
        5. run visionMotion
        */
        addSequential(new GetTableData());
        if (Robot.netWorkTable.angle < 0){
            addSequential(new TurnLeft(Robot.netWorkTable.angle));
        } else if(Robot.netWorkTable.angle > 0){
            addSequential(new TurnRight(Robot.netWorkTable.angle));
        } else if (Robot.netWorkTable.angle == 0){
        }
        addSequential(new MoveY(Robot.netWorkTable.distance));
        if (Robot.netWorkTable.angle < 0){
            addSequential(new TurnLeft(Robot.netWorkTable.angle));
        } else {
            addSequential(new TurnRight(Robot.netWorkTable.angle));
        }
    }
}