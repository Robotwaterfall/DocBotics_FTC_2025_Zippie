package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.teamcode.Constants.shooterConstants.shooterPhysicsConstants.WHEEL_RADIUS_INCH;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class flywheelSubsystem extends SubsystemBase {

    DcMotorEx flyWheelMotor;

    public flywheelSubsystem(DcMotorEx flyWheelMotor){
        this.flyWheelMotor = flyWheelMotor;

        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public DcMotorEx getFlyWheelMotor(){


        return flyWheelMotor;
    }

    public void setMotorTangentialVelocity(double tangentialVelocity){
        double output = tangentialVelocity / WHEEL_RADIUS_INCH;
        flyWheelMotor.setVelocity(output , AngleUnit.RADIANS);

    }

    public double getMotorTangentialVelocity(){
        double output = flyWheelMotor.getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS_INCH;
        return output;
    }
}
