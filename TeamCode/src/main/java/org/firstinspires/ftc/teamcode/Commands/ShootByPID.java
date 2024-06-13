package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.MMLib.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class ShootByPID extends MMPIDCommand {

    private final double setPoint;

    public ShootByPID(double setPoint) {
        super(MMRobot.getInstance().mmSystems.shooterPID, setPoint);
        this.setPoint = setPoint;
    }

    @Override
    public void initialize() {
        super.initialize();
        MMRobot.getInstance().mmSystems.telemetry.addData("Set Point", setPoint);
    }
}
