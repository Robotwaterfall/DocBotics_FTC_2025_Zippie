package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.goalHeightInches;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.limelightLensHeightInches;
import static org.firstinspires.ftc.teamcode.Constants.limelightConstants.limelightMountAngleDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public double getDistanceInch(){
        double targetOffSetAngle_Vertical = getTx();

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffSetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        TelemetryPacket llpacket = new TelemetryPacket();

        if (result != null && result.isValid()) {
            llpacket.put("Tx", result.getTx());
            llpacket.put("Ty", result.getTy());
            llpacket.put("Ta", result.getTa());
        }

        dashboard.sendTelemetryPacket(llpacket);
    }

}