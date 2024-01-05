package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.ArmPosition;

public class ArmPositionSelector {
    ArmPosition[][] sideScorePositions = {
            {ArmPosition.SCORE_BOTTOM_CLOSE, ArmPosition.SCORE_BOTTOM_FRONT, ArmPosition.SCORE_BOTTOM_FAR},
            {ArmPosition.SCORE_MID_CLOSE, ArmPosition.SCORE_MID_FRONT, ArmPosition.SCORE_MID_FAR},
            {ArmPosition.SCORE_TOP_CLOSE, ArmPosition.SCORE_TOP_FRONT, ArmPosition.SCORE_TOP_FAR}
    };
    String[] robotPosition = {"Left", "Front", "Right"};
    /*
        TODO: Left  and Right - X can be 0 or 2 (close or far)
              Front - X can be only 1 (Front). which means only Y can change
              These limitations needs to happen in the move methods
              for example if the position is Left then jump 2 each press in the code.
              do these changes limitations in the code only,
              the telemetry view has to stay simple, and constant. In order to not confuse the 2nd driver.
              In telemetry add another line that indicates which column the user can use to score
              for example use ^ under the XXX
              use buttons to choose which position the robot is in: Left, Front or Right.
    */


    public static int Y = 1;
    public static int X = 0;
    public static String position;

    public static void moveRight() {
        if (X != 2) {
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
        return sideScorePositions[Y][X];
    }

    public static void telemetry(Telemetry telemetry) {
        for (int yCounter = 0; yCounter < 3; yCounter++) {
            for (int xCounter = 0; xCounter < 3; xCounter++) {
                String tempStr = "";
                if(xCounter == X && yCounter == Y) {
                    tempStr += "O";
                } else {
                    tempStr += "X";
                }
                telemetry.addLine(tempStr);
            }
        }

        telemetry.update();
    }
}

