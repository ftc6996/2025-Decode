package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
    Drivetrain drivetrain;
    DoubleSupplier forward, strafe, turn;

    public Drive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn)
    {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        addRequirements(drivetrain);
    }

    @Override
    public void execute()
    {
        drivetrain.drive(forward, strafe, turn);
    }
}
