package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Side;

public class ArmPositionSelector {
    public static ArmPosition[][] sideScorePositions = {
            {ArmPosition.SCORE_TOP_CLOSE, ArmPosition.SCORE_TOP_FAR},
            {ArmPosition.SCORE_MID_CLOSE, ArmPosition.SCORE_MID_FAR},
            {ArmPosition.SCORE_BOTTOM_CLOSE, ArmPosition.SCORE_BOTTOM_FAR} //TODO SCORE_BOTTOM_CLOSE
    };
    public static ArmPosition[] frontScorePositions = {ArmPosition.SCORE_TOP_FRONT, ArmPosition.SCORE_FIRST_CYCLE_LOW_RED, ArmPosition.SCORE_FIRST_CYCLE_LOW_BLUE};
    private static int selectedScoreHeight = 1;
    private static boolean isSelectedScoreSideLeft = false;
    private static Side selectedRobotSide = Side.CENTER;

    public static void setRobotSide(Side side) {
        selectedRobotSide = side;
    }

    public static Side getSelectedRobotSide() {
        return selectedRobotSide;
    }

    public static void moveRight() {
        isSelectedScoreSideLeft = false;}

    public static void moveLeft() {
        isSelectedScoreSideLeft = true;}

    public static void moveUp() {
        if (selectedScoreHeight > 0) {
            selectedScoreHeight--;
        }
    }

    public static void moveDown() {
        if (selectedScoreHeight < 2) {
            selectedScoreHeight++;
        }
    }

    public static ArmPosition getPosition() {
        if (selectedRobotSide == Side.CENTER) {
            return frontScorePositions[selectedScoreHeight];
        }
        boolean isClose = ((selectedRobotSide == Side.LEFT) == isSelectedScoreSideLeft);
        return sideScorePositions[selectedScoreHeight][isClose?0:1];
    }

    public static void telemetry(Telemetry telemetry) {
        for (int yCounter = 0; yCounter < 3; yCounter++) {
            StringBuilder tempStr = new StringBuilder();
            for (int xCounter = 0; xCounter < 2; xCounter++) {
                if (xCounter == (isSelectedScoreSideLeft?0:1) && yCounter == selectedScoreHeight) {
                    tempStr.append("X");
                } else {
                    tempStr.append("O");
                }
            }
            telemetry.addLine(tempStr.toString());
        }
        if (selectedRobotSide.equals(Side.RIGHT)) {
            telemetry.addLine("__^");
        } else if (selectedRobotSide.equals(Side.CENTER)) {
            telemetry.addLine("_^_");
        } else {
            telemetry.addLine("^__");
        }
    }

    public static boolean getIsLeftOfBoard() {
        return selectedRobotSide != Side.RIGHT;
    }
}

