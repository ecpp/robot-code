package Connexion;

import UWB.helpers.Point2D;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ThreadedBehaviourFactory;
import jade.lang.acl.ACLMessage;
import static jade.lang.acl.ACLMessage.INFORM;
import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.sensors.ev3.EV3TouchSensor;
import ev3dev.sensors.ev3.EV3UltrasonicSensor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;
import UWB.pozyx.TagIdMqtt;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.Stack;


public class Agent2 extends Agent {
    TagIdMqtt tag;
    {
        try {
            tag = new TagIdMqtt("6a75");
        } catch (MqttException e) {
            throw new RuntimeException(e);
        }
    }

    private Point2D position = new Point2D(0, 0);
    private Stack<Point2D> targets = new Stack<>();
    private Point2D rechargingPoint = new Point2D(14357, 15183);
    //14357 15183
    private double yaw = 0;
    private static final int CONSTANT_BATTERY_TIME = 600; // Quarter Seconds
    private int artificialBatteryTime = CONSTANT_BATTERY_TIME;
    private boolean isRunning = true;
    private final EV3TouchSensor batteryRechargeSensor = new EV3TouchSensor(SensorPort.S4);
//    private final EV3TouchSensor stopButtonSensor = new EV3TouchSensor(SensorPort.S2);
    private final EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
    private final EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
    private final EV3LargeRegulatedMotor motorGrabber = new EV3LargeRegulatedMotor(MotorPort.B);
    private final EV3UltrasonicSensor ultrasonicSensorRight = new EV3UltrasonicSensor(SensorPort.S1);
    private final EV3UltrasonicSensor ultrasonicSensorLeft = new EV3UltrasonicSensor(SensorPort.S3);

    private final EV3UltrasonicSensor ultrasonicSensorMiddle = new EV3UltrasonicSensor(SensorPort.S2);
    private int lastError1 = 0;
    private int lastError2 = 0;
    private int cumError1 = 0;
    private int cumError2 = 0;
    private long previousTime;
    private float[] distanceValues = new float[3];


    @Override
    protected void setup() {
        initializeTargets();
        addBehaviour(new InitializationBehaviour());
        addBehaviour(new MovementBehaviour());
        addBehaviour(new ThreadedBehaviour().wrap(new BatteryCheckBehaviour()));
//        addBehaviour(new ThreadedBehaviour().wrap(new RunningCheckBehaviour()));
    }

    private void initializeTargets() {
        while (distanceValues[0] == 0 || distanceValues[1] == 0 || distanceValues[2] == 0){
            Delay.msDelay(10);
            distanceValues = checkDistance();
        }
        Point2D target2 = new Point2D(13690, 13690);
        Point2D target1 = new Point2D(10450, 14900);
        targets.push(target2);
        targets.push(target1);
    }

    private class ThreadedBehaviour extends ThreadedBehaviourFactory {

    }

    private class BatteryCheckBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            if (batteryRechargeSensor.isPressed()) {
                System.out.println("Recharging battery, please wait.");
                artificialBatteryTime = CONSTANT_BATTERY_TIME;
                Delay.msDelay(2000);
                resetPID();
            }
            else if (artificialBatteryTime == 480 && !targets.empty() && targets.peek() != rechargingPoint) {
                System.out.println("Low battery going to charge.");
                targets.push(rechargingPoint);
            }
            else if (targets.empty()){
                System.out.println("Finished job going to charging dock");
                targets.push(rechargingPoint);
            }
            else if (artificialBatteryTime > 0) {
                if (isRunning) {
                    artificialBatteryTime--;
                } else {
                    System.out.println("Battery on hold");
                }
                position = tag.getLocation();
                Delay.msDelay(250);
            }
            System.out.println(artificialBatteryTime);
            Delay.msDelay(1);
        }
    }
    

    private class MovementBehaviour extends CyclicBehaviour {
        private double calculateTurnAngle(Point2D target){
            try{
                yaw = Math.toDegrees(tag.getYaw());
//                System.out.println("yaw " + yaw);
//                System.out.println("location: "+ position.x + " --- "+position.y);
                double angleToTarget = Math.toDegrees(Math.atan2(target.y - position.y,
                        target.x - position.x));

                // Normalize the angleToTarget to a 0-360 range
                angleToTarget = (angleToTarget + 360) % 360;

                // Calculate the difference
                double angleDifference = angleToTarget - yaw;

                // Normalize the angleDifference to a -180 to 180 range
                angleDifference = (angleDifference + 180) % 360 - 180;
                //System.out.println("angle"+angleDifference);
                return angleDifference;
            }
            catch (Exception e){
                System.out.println(e);
            }


            return 0;
        }

        private boolean isArrived(Point2D target){
            try{
                double xdiff = Math.abs(target.x - position.x);
                double ydiff = Math.abs(target.y - position.y);
                double result = Math.sqrt((ydiff*ydiff) - (xdiff * xdiff));

                if (result < 75){
                    return true;
                }
                return false;
            }
            catch (Exception e) {
                System.out.println(e);
                return false;
            }

        }

        @Override
        public void action() {


            distanceValues = checkDistance();
            if (isRunning && artificialBatteryTime > 0) {
                if (targets.empty()){
                    System.out.println("Waiting for the target.");
                    stopMotors();
                    resetPID();
                }
                else if (isArrived(targets.peek())){
                    System.out.println("reached to destination.");
                    stopMotors();
                    Delay.msDelay(50);

                    try{
                        if(targets.peek() == rechargingPoint) {
                            System.out.println("Recharging battery at location.");
                            artificialBatteryTime = 600;
                        }
                        else{
                            operateGrabber();
                        }
                        targets.pop();
                    }
                    catch (Exception e){
                        System.out.println(e);
                    }
                    resetPID();
                }
            else{

                    if (distanceValues[0] > 20 && distanceValues[1] > 20 && distanceValues[2] > 30)
                    {

                        double turnangle = calculateTurnAngle(targets.peek());
                        double abs = Math.abs(turnangle);
                        if (abs > 15 && abs < 165){
                            if (turnangle < 0){
                                System.out.println("Turning right to target.");
                                motorRight.setSpeed(30);
                                motorLeft.setSpeed(30 + (int)(abs * 4));
                            }
                            else{
                                System.out.println("Turning left to target.");
                                motorRight.setSpeed(30 + (int)(abs * 4));
                                motorLeft.setSpeed(30);
                            }
                        }
                        else{
                            if(distanceValues[2] > 100) {
                                motorRight.setSpeed(1000);
                                motorLeft.setSpeed(1000);

                            }
                            else{
                                motorRight.setSpeed((int)distanceValues[2]*8);
                                motorLeft.setSpeed((int)distanceValues[2]*8);
                            }
                            System.out.println("going straight to target.");
                        }

                        motorRight.forward();
                        motorLeft.forward();


                    }
                    else{
                        calculateAndSetMotorSpeeds();
                    }


                }
                }

            else {
                stopMotors();
                resetPID();
            }
        }


        private void operateGrabber() {
            motorGrabber.rotate(750);
            Delay.msDelay(500);
            motorGrabber.rotate(-750);
        }

        private void calculateAndSetMotorSpeeds() {
            long currentTime = System.currentTimeMillis();
            long elapsedTime = currentTime - previousTime;
            if (elapsedTime == 0){
                elapsedTime = 1;
            }
            int setPoint = 0;
            int error1 = (int) (setPoint - distanceValues[0]);
            int error2 = (int) (setPoint - distanceValues[1]);

            cumError1 += error1 * elapsedTime;
            cumError2 += error2 * elapsedTime;

            int rateError1 = (int) ((error1 - lastError1) / elapsedTime);
            int rateError2 = (int) ((error2 - lastError2) / elapsedTime);

            int output1 = calculateMotorOutput(error1, cumError1, rateError1);
            int output2 = calculateMotorOutput(error2, cumError2, rateError2);

            updateMotorSpeeds(output1, output2);

            lastError1 = error1;
            lastError2 = error2;
            previousTime = currentTime;
        }

        private int calculateMotorOutput(int error, int cumError, int rateError) {
            float kd = 3.2F;
            float ki = 0.00001F;
            float kp = 1.6F;
            return (int) (kp * error + ki * cumError + kd * rateError);
        }

        private void updateMotorSpeeds(int output1, int output2) {
            output1 = adjustOutput(output1);
            output2 = adjustOutput(output2);

            if (shouldMoveStraight()) { //kaldir!
                motorRight.setSpeed(Math.max(output1, output2));
                motorLeft.setSpeed(Math.max(output1, output2));
            } else {
                if (output2> output1)
                    output2 = output2 * 2;
                else
                    output1 = output1 * 2;
                motorRight.setSpeed(output2);
                motorLeft.setSpeed(output1);
            }

            motorRight.forward();
            motorLeft.forward();
        }

        private int adjustOutput(int output) {
            return Math.abs(output) * 2 + 200;
        }

        private boolean shouldMoveStraight() {
            return Math.abs(distanceValues[0] - distanceValues[1]) <= 50 && distanceValues[0] > 100 && distanceValues[1] > 100;
        }
    }

    private float[] checkDistance() {
        float[] sampleLeft = new float[ultrasonicSensorLeft.sampleSize()];
        float[] sampleRight = new float[ultrasonicSensorRight.sampleSize()];
        float[] sampleMiddle = new float[ultrasonicSensorMiddle.sampleSize()];

        ultrasonicSensorLeft.fetchSample(sampleLeft, 0);
        ultrasonicSensorRight.fetchSample(sampleRight, 0);
        ultrasonicSensorMiddle.fetchSample(sampleMiddle, 0);

        if (sampleLeft[0] > 500){
            sampleLeft[0] = 500;
        }
        if (sampleRight[0] > 500){
            sampleRight[0] = 500;
        }
        if (sampleMiddle[0] > 500){
            sampleMiddle[0] = 500;
        }
        return new float[]{sampleRight[0], sampleLeft[0], sampleMiddle[0]};
    }

    private void resetPID(){
        lastError1 = 0;
        lastError2 = 0;
        cumError1 = 0;
        cumError2 = 0;
        previousTime = System.currentTimeMillis();
    }

    private class InitializationBehaviour extends OneShotBehaviour {
        @Override
        public void action() {
            sendInitializationMessage();
            System.out.println("Initialization successful!");
            previousTime = System.currentTimeMillis();
        }

        private void sendInitializationMessage() {
            try {
                ACLMessage message = new ACLMessage(INFORM);
                message.addReceiver(new AID("Agent1@192.168.0.122:1099/JADE", AID.ISGUID));
                message.setContent("Initialization successful!");
                send(message);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private void stopMotors() {
        motorLeft.setSpeed(0);
        motorRight.setSpeed(0);
        motorLeft.stop(true);
        motorRight.stop(true);
        Delay.msDelay(500);
    }

    @Override
    protected void takeDown() {
        super.takeDown();
        stopMotors();
    }
}