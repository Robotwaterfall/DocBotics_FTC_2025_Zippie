package org.firstinspires.ftc.teamcode.Command;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.transferSubsystem;

public class teleOpTransferCommand extends CommandBase {

    transferSubsystem transferSubsystem;
    DcMotor transferMotor;
    double transferSpeed;

    private ElapsedTime runTime = new ElapsedTime();
    private final double m_timeIndexing;



    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public teleOpTransferCommand(transferSubsystem transferSubsystem,
                                 double transferSpeed,
                                 double timeIndexing){
        this.transferSubsystem = transferSubsystem;
        this.transferMotor = transferSubsystem.getTransferMotor();
        this.transferSpeed = transferSpeed;
        this.m_timeIndexing = timeIndexing;
        addRequirements(transferSubsystem);

    }

    @Override
    public void initialize() {
        transferMotor.setPower(0);
        runTime.reset();

    }

    @Override
    public void execute() {


        transferMotor.setPower(transferSpeed);


    }

    @Override
    public boolean isFinished() {
        boolean isFinishedTransfering = runTime.seconds() >= m_timeIndexing;
        return isFinishedTransfering;
    }

    @Override
    public void end(boolean interrupted) {
        transferMotor.setPower(0);

    }

}