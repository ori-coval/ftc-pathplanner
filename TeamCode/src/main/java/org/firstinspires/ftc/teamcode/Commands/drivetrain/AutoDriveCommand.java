package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.AutoDriveTrain;

import java.util.function.DoubleSupplier;

public class AutoDriveCommand extends CommandBase {

    private final AutoDriveTrain drive;
    private final DoubleSupplier leftY, leftX, rightX;

    public AutoDriveCommand(AutoDriveTrain drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
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