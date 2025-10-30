package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    private Telemetry telemetry;
    private Drivetrain drivetrain;
    public TelemetrySubsystem(Drivetrain drivetrain, Telemetry telemetry)
    {
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void periodic()
    {
        // Telemetry Data for Drivetrain
        telemetry.addLine("DriveTrain");
        telemetry.addData("Version", Drivetrain.Version);

        telemetry.update();
    }
}
