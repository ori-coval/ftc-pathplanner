package org.firstinspires.ftc.teamcode.MMLib.Examples.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;

@TeleOp
public class EncoderTest extends MMTeleOp {

    public EncoderTest() {
        super(false);
    }

    DcMotorEx motorEx;

    @Override
    public void main() {

        motorEx = hardwareMap.get(DcMotorEx.class, "shooter");

        addRunnableOnInit(
                () -> motorEx.setPower(0.5)
        );

    }

    @Override
    public void run() {
        telemetry.addData("encoder value", motorEx.getCurrentPosition());
        super.run();
    }
}
