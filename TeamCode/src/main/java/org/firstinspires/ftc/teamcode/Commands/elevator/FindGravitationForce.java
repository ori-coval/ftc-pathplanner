package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class FindGravitationForce extends CommandBase {
    double power = 0;
    Elevator elevator;
    Telemetry telemetry;
    public FindGravitationForce(Elevator elevator){
        this.elevator = elevator;
        this.telemetry = telemetry;
    }
    @Override
    public void execute() {
        if (elevator.height - previousHeight){
            power += 0.02;
        }
        elevator.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return elevator.height != previousHeight;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
        telemetry.addData("telemetry", )
        telemetry.update();
    }
}
