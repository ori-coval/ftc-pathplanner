package org.firstinspires.ftc.teamcode.Commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class FindGravitationForce extends CommandBase {
    private double power = 0;
    private Elevator elevator;
    private Telemetry telemetry;
    private double previousHeight = 0;
    private final double THRESHOLD = 0.2;
    public FindGravitationForce(Elevator elevator){
        this.elevator = elevator;
        this.telemetry = telemetry;
        addRequirements(elevator);
    }
    public boolean isHeightSimilar(){
        return Math.abs(elevator.getHeight() - previousHeight) < THRESHOLD;
    }
    @Override
    public void execute() {
        if (isHeightSimilar()){
            power += 0.02;
        }
        previousHeight = elevator.getHeight();
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
    }
}
