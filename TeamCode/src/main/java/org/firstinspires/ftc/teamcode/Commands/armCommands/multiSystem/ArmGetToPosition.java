package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToPosition extends ConditionalCommand {
    public static ArmPosition lastPosition = ArmPosition.INTAKE;
    private final ArmPosition targetPosition;
    public ArmGetToPosition(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new UnsafeMoveArm(robot, position, isLeftOfBoard),
                new UnsafeMoveArm(robot, ArmPosition.SAFE_PLACE, isLeftOfBoard).andThen(
                        new UnsafeMoveArm(robot, position, isLeftOfBoard)
                ),
                () -> lastPosition.getCluster() == position.getCluster()
        );
        targetPosition = position;
    }


    @Override
    public void initialize() {
        super.initialize();
        lastPosition = targetPosition;
    }

    @Override
    public void execute() {
        super.execute();
        FtcDashboard.getInstance().getTelemetry().addData("isArmGetToPositionFinished", isFinished());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
