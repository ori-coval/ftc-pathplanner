package org.firstinspires.ftc.teamcode.MMLib.PID;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MMRobot;

public class MMTuningFFCommand extends CommandBase {

    MMRobot mmRobot = MMRobot.getInstance();

    private final MMPIDSubsystem subsystem;

    private double power = 0;
    private double jumps = 0.0002;
    private final double threshold;
    private double startingPoint = 0;

    public MMTuningFFCommand(MMPIDSubsystem subsystem, double threshold) {
        this.subsystem = subsystem;
        this.threshold = threshold;
        addRequirements(subsystem);
    }

    public MMTuningFFCommand(MMPIDSubsystem subsystem, double threshold, double powerJumps) {
        this(subsystem, threshold);
        jumps = powerJumps;
    }

    public MMTuningFFCommand(MMPIDSubsystem subsystem, double threshold, double powerJumps, double startingPoint) {
        this(subsystem, threshold, powerJumps);
        this.startingPoint = startingPoint;
    }

    private boolean isThresholdReached() {
        return !(Math.abs(subsystem.getCurrentValue() - startingPoint) < threshold);
    }

    @Override
    public void execute() {
        /*add telemetry*/

        if(!isThresholdReached()) {
            power += jumps;
        }
        subsystem.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return isThresholdReached();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop(); //this might wouldn't work if the user didn't override it
        /*add telemetry of minPower*/
        power = 0;
    }
}
