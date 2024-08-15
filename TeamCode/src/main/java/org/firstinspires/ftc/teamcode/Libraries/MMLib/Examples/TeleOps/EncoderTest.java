package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class EncoderTest extends MMTeleOp {

    public EncoderTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    DcMotorEx motorEx;

    @Override
    public void onInit() {

        motorEx = hardwareMap.get(DcMotorEx.class, "shooter");

        addRunnableOnInit(
                () -> motorEx.setPower(0.5)
        );

    }

    @Override
    public void run() {
        telemetry.addData("encoder value", motorEx.getCurrentPosition());
        telemetry.update();
    }
}
