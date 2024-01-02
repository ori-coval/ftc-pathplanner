package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.ArmPosition;

public class ArmPositionSelector {
    private Telemetry telemetry;


    ArmPosition[][] sideScorePositions={
            {ArmPosition.SCORE_TOP_CLOSE, ArmPosition.SCORE_TOP_FAR, ArmPosition.SCORE_FRONT_SCORE_TOP},
            {ArmPosition.SCORE_MID_CLOSE, ArmPosition.SCORE_MID_FAR, ArmPosition.SCORE_FRONT_SCORE_MID},
            {ArmPosition.SCORE_BOTTOM_CLOSE, ArmPosition.SCORE_BOTTOM_FAR, ArmPosition.SCORE_FRONT_SCORE_BOTTOM}
    };
    int Y = 1;
    int X = 1;

    public void moveXRight() {
        if (!(X == 2)) {
            X++;
        }
    }
    public void moveXLeft() {
        if (!(X == 0)) {
            X--;
        }
    }
    public void moveYRight() {
        if (!(Y == 2)) {
            Y++;
        }
    }
    public void moveYLeft() {
        if (!(Y == 0)) {
            Y--;
        }
    }
    public ArmPosition getPosition() {
        return sideScorePositions[X][Y]; //TODO
    }
}

