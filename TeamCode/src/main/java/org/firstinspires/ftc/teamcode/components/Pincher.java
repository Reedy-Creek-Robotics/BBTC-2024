package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Robot;

public class Pincher {
    private Servo pincher;

    public Pincher(HardwareMap hardwareMap){
        pincher = hardwareMap.get(Servo.class, "pincher");
    }

    public class PincherOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pincher.setPosition(Robot.PINCHER_OPEN);
            return false;
        }
    }

    public Action pincherOpen(){
        return new Pincher.PincherOpen();
    }

    public class PincherClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pincher.setPosition(Robot.PINCHER_CLOSED);
            return false;
        }
    }

    public Action pincherClose(){
        return new Pincher.PincherClose();
    }
}