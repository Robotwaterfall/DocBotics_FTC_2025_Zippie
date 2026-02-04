package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class limelightSubsystem extends SubsystemBase {

    Limelight3A limelight;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public limelightSubsystem(HardwareMap hardwareMap){
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }
    public boolean hasTarget(){
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }
    public double getTx(){ //horizontal offset in degrees
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r.getTx() : -1.0; //-1 means no target
    }

    public double getTy(){
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r.getTy() : -1.0; //-1 mean no target
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        TelemetryPacket llpacket = new TelemetryPacket();

        if (result != null && result.isValid()) {
            llpacket.put("Tx", result.getTx());
            llpacket.put("Ty", result.getTy());
            llpacket.put("Ta", result.getTa());
        } else {
            llpacket.put("Limelight", "No valid target");
        }

        dashboard.sendTelemetryPacket(llpacket);
    }

}