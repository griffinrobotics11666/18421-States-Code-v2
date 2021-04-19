package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoEx {
    public Servo servo;
    private double desiredPosition;
    private double speed = 0.0;
    private double lastUpdateTimestamp = 0.0;
    private NanoClock clock;

    public ServoEx(Servo servo){
        this.servo = servo;
        clock = NanoClock.system();
    }

    public void setPosition(double position){
        servo.setPosition(position);
        desiredPosition = position;
    }

    public void asyncGoToPosition(double position, double speed){
        desiredPosition = position;
        this.speed = speed;
    }

    public void asyncGoToPosition(double position){
        if(speed != 0.0){
            desiredPosition = position;
        }
    }

    public void setSpeed(double speed){
        this.speed = Math.abs(speed);
    }

    public void update(){
        if(lastUpdateTimestamp > 0.0){
            double elapsedTime = clock.seconds()-lastUpdateTimestamp;
            if(Math.abs(servo.getPosition() - desiredPosition) < elapsedTime*speed){
                setPosition(desiredPosition);
            }
            else {
                setPosition(servo.getPosition() + (elapsedTime*speed*Math.signum(desiredPosition-servo.getPosition())));
            }
        }
        lastUpdateTimestamp = clock.seconds();
    }

    public boolean isFinished() {
        return desiredPosition == servo.getPosition();
    }

    public double getDesiredPosition() {
        return desiredPosition;
    }

    public double getSpeed(){
        return speed;
    }

    public void goToPosition(double position, double speed){
        asyncGoToPosition(position, speed);
        while(!isFinished()){
            update();
        }
    }

    public void goToPosition(double position){
        asyncGoToPosition(position);
        while(!isFinished()){
            update();
        }
    }
}
