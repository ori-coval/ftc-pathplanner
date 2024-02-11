package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ScoringFirstPixel extends SequentialCommandGroup {
    public ScoringFirstPixel(Cartridge cartridge,Elevator elevator, Extender extender, Elbow elbow, Turret turret, AntiTurret antiTurret, Side side, SampleMecanumDrive driveTrain){
        super(

                new TrajectoryFollowerCommand(Trajectories.get("Driving to score from the stack to the back stage for side score"), driveTrain),
                new SideCommandSwitch(
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_BOTTOM_CLOSE, true),
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_BOTTOM_FAR, true),
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_MID_FAR, true),
                        () -> side),
                new CartridgeSetState(cartridge,Cartridge.State.OPEN)

        );
    }

}
