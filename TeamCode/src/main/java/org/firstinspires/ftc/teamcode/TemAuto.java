package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Temp Auto")
public class TemAuto {

    public void runOpMode() {

        Hardware hw = Hardware.getInstance();
        hw.init.HardwareMap();

    }

}
