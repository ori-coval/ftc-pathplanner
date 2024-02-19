package org.firstinspires.ftc.teamcode.Commands.armCommands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class FindGravitationForce extends CommandBase {
    private double power = 0;
    private Elevator elevator;
    private Telemetry telemetry;
    private final double THRESHOLD = 1;
    public FindGravitationForce(Elevator elevator, Telemetry telemetry){
        this.elevator = elevator;
        this.telemetry = telemetry;
        addRequirements(elevator);
    }
    public boolean isHeightSimilar(){
        return Math.abs(elevator.getHeight() - 0) < THRESHOLD;
    }
    @Override
    public void execute() {
        telemetry.addData("kg test", 1);
        telemetry.addData("kg power", power);
        telemetry.update();
        if (isHeightSimilar()){
            power += 0.0002;
        }
        elevator.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return !isHeightSimilar();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
        telemetry.addData("min power", power);
        telemetry.update();
        power = 0;
    }
}
