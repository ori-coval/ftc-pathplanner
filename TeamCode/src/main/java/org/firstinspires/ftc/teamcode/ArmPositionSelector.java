package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.ArmPosition;

public class ArmPositionSelector {
    private Telemetry telemetry;


    /*



     */

    ArmPosition[][] sideScorePositions = {
            {ArmPosition.SCORE_TOP_CLOSE, ArmPosition.SCORE_TOP_FAR, ArmPosition.SCORE_FRONT_SCORE_TOP},
            {ArmPosition.SCORE_MID_CLOSE, ArmPosition.SCORE_MID_FAR, ArmPosition.SCORE_FRONT_SCORE_MID},
            {ArmPosition.SCORE_BOTTOM_CLOSE, ArmPosition.SCORE_BOTTOM_FAR, ArmPosition.SCORE_FRONT_SCORE_BOTTOM}
    };
    public static int Y = 1;
    public static int X = 1;

    public static void moveRight() {
        if (!(X == 2)) {
            X++;
        }
    }
    public static void moveLeft() {
        if (!(X == 0)) {
            X--;
        }
    }
    public static void moveUp() {
        if (!(Y == 2)) {
            Y++;
        }
    }
    public static void moveDown() {
        if (!(Y == 0)) {
            Y--;
        }
    }
    public ArmPosition getPosition() {
        return sideScorePositions[X][Y]; //TODO
    }
}

