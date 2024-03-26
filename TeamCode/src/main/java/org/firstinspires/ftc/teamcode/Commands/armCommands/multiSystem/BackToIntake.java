package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class BackToIntake extends ParallelCommandGroup {
    public BackToIntake(RobotControl robot) {
        super(
                new ArmGetToPosition(robot, ArmPosition.DRIVING_INTAKE, true)
//                new WaitCommand(50).andThen(new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)) //ready for next pixel
        );
    }
}
