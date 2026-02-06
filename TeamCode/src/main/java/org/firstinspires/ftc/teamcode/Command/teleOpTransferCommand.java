package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.ShooterLockedZone;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.tyOffSet;
import static org.firstinspires.ftc.teamcode.Constants.llLockOnKp;
import static org.firstinspires.ftc.teamcode.Constants.lockOnDeadband;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.transferSubsystem;

public class teleOpTransferCommand extends CommandBase {

    transferSubsystem transferSubsystem;
    limelightSubsystem llSub;
    DcMotor transferMotor;
    double transferSpeed;
    boolean isAuto = false;

    private final double m_timeIndexing;
    private ElapsedTime runtime = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public teleOpTransferCommand(transferSubsystem transferSubsystem,
                                 limelightSubsystem llSub,
                                 double transferSpeed,
                                 double timeIndexing,
                                 boolean isAuto){
        this.transferSubsystem = transferSubsystem;
        this.llSub = llSub;
        this.transferMotor = transferSubsystem.getTransferMotor();
        this.transferSpeed = transferSpeed;
        this.m_timeIndexing = timeIndexing;
        this.isAuto = isAuto;
        addRequirements(transferSubsystem);

    }

    @Override
    public void initialize() {
        transferMotor.setPower(0);
        runtime.reset();

    }

    @Override
    public void execute() {

        boolean isLockedOn = Math.abs(llSub.getTy() - tyOffSet) < lockOnDeadband;
        if(llSub.hasTarget() && isLockedOn || isAuto) {
            transferMotor.setPower(transferSpeed);
        }

        TelemetryPacket drivePacket = new TelemetryPacket();
        drivePacket.put("isLockedOn", isLockedOn);
        dashboard.sendTelemetryPacket(drivePacket);

    }

    @Override
    public boolean isFinished() {
        return runtime.seconds() >= m_timeIndexing;
    }

    @Override
    public void end(boolean interrupted) {
        transferMotor.setPower(0);

    }

}