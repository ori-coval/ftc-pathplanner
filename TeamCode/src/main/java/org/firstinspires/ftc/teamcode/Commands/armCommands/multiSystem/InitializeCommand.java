package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToAngle;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public class InitializeCommand extends SequentialCommandGroup {
    public InitializeCommand(RobotControl robot) {
        super(
                new ExtenderSetPosition(robot.extender, Extender.Position.CLOSED),
                new AntiTurretGetToAngle(robot.antiTurret, ArmPosition.INTAKE.getAntiTurretAngle(true)),
                new UnsafeMoveArm(robot, ArmPosition.INTAKE, false)
        );
    }
}
