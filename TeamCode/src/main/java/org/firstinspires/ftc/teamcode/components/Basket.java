package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Robot;

public class Basket {
    private Servo basket;

    public Basket(HardwareMap hardwareMap){
        basket = hardwareMap.get(Servo.class, "basket");
    }

    public class BasketUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            basket.setPosition(Robot.BASKET_UP);
            return false;
        }
    }

    public Action basketUp(){
        return new Basket.BasketUp();
    }

    public class BasketDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            basket.setPosition(Robot.BASKET_DOWN);
            return false;
        }
    }

    public Action basketDown(){
        return new Basket.BasketDown();
    }
}