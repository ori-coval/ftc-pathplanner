package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private final DriveTrain drive;
    private final DoubleSupplier leftY, leftX, rightX;

    public DriveCommand(DriveTrain drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble());
    }

}