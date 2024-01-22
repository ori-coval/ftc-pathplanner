package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class CollectingFromStack extends CommandBase {
    InTake inTake;
    Conveyor conveyor;

    public CollectingFromStack(InTake inTake){
        this.inTake = inTake;
    }
    private void collecting(){
        inTake.setPower(inTake.COLLECT_POWER);
        inTake.setStackPosition(2);
        inTake.setStackPosition(1);
        inTake.setStackPosition(0);
    }
    @Override
    public void initialize() {
     if (!conveyor.isRobotFull()) {
         if (inTake.getStackPosition() == 3) {
             collecting();
         } else {
             inTake.setStackPosition(3);
         }
     }
   }
}
