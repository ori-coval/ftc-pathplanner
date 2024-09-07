package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

public class TFISGOINGON extends MMTeleOp {

    public TFISGOINGON() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        telemetry.addLine("HAII");
        telemetry.update();
    }

}
