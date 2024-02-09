package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

public class TeleopDriveCommand extends CommandBase {
    private final DriveTrain driveTrain;
    private final Gamepad gamepad;

    public TeleopDriveCommand(DriveTrain driveTrain, Gamepad gamepad){
        this.driveTrain = driveTrain;
        this.gamepad = gamepad;
        this.addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        // movement direction
        Vector2d joyStickDirection = new Vector2d(gamepad.left_stick_x, -gamepad.left_stick_y);

        // make it field oriented
        Vector2d fieldOrientedVector = joyStickDirection.rotateBy(-driveTrain.getYawInDegrees());

        // execute it using the drive train
        driveTrain.drive(fieldOrientedVector, gamepad.right_stick_x);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new Vector2d(0, 0), 0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}