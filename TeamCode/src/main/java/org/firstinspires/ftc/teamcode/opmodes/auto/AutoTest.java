package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "goaltest")
public class AutoTest extends NextFTCOpMode {
    private Timer timer1 = new Timer();
    private Timer timer2 = new Timer();
    public int state;
    public AutoTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left");
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed();
    private final MotorEx backLeftMotor = new MotorEx("back_left");
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed();

    public void runAllMotors(double fl, double fr, double bl, double br) {
        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    public void zeroMotors() {
        runAllMotors(0,0,0,0);
    }

    @Override
    public void onStartButtonPressed() {
        timer1.resetTimer();
        timer2.resetTimer();
        runAllMotors(-0.5,-0.5,-0.5,-0.5);
    }

    @Override
    public void onUpdate() {
        switch (state) {
            case 1:
                if(timer1.getElapsedTimeSeconds() > 1) {
                    runAllMotors(-0.5,0.5,-0.5,0.5);
                    timer2.resetTimer();
                    state = 2;
                }
            case 2:
                if (timer2.getElapsedTimeSeconds() > 1) {
                    runAllMotors(0,0,0,0);
                    state = 3;
                }
            case 3:

        }
    }
}
