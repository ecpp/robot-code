package Connexion;

import UWB.helpers.Point2D;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.util.ExtendedProperties;
import jade.util.leap.Properties;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import lejos.utility.Delay;
import UWB.pozyx.TagIdMqtt;
import org.eclipse.paho.client.mqttv3.MqttException;



public class container2 {
    static TagIdMqtt our_tag;
    static TagIdMqtt target_tag;
    private static Point2D position = new Point2D(0, 0);
    public static void main(String[] args) {

        try {
            Runtime runtime = Runtime.instance();
//            String target ="192.168.0.122";
            String target ="192.168.0.133";
            String source ="192.168.0.126";
            ProfileImpl p = new ProfileImpl(target,1099,null,false);

            p.setParameter(Profile.LOCAL_HOST,source);
            p.setParameter(Profile.LOCAL_PORT,"1099");

            AgentContainer agentContainer=runtime.createAgentContainer(p);
            Connexion.container2.start();


            AgentController agent2=agentContainer.createNewAgent("Agent2",
                    "Connexion.Agent2",new Object[]{});
            boolean is_connected = false;
            while (!is_connected){
                {
                    try {
                        our_tag = new TagIdMqtt("6a75");
                        //target_tag = new TagIdMqtt("684e");
                        target_tag = new TagIdMqtt("682e");
                        is_connected = true;
                    } catch (MqttException e) {
                        System.out.println("Could not connect trying again.");
                        //throw new RuntimeException(e);
                        System.out.println(e);
                    }
                }
            }

            System.out.println("starting agent");
            agent2.start();
            while (true) {
                try{
                    boolean isCollided = check_preventCollision();
                    System.out.println("while true");
                    if (isCollided) {
                        System.out.println("We Collided!!!");
                        //agent2.kill();
                    }
                }
                catch (Exception e){
                    System.out.println(e);
                }
                Delay.msDelay(10);

            }
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private static void start() {
    }


    public static boolean check_preventCollision() {
        Point2D target_position = target_tag.getLocation();

        //double angle1 = our_tag.getYaw();
        position = our_tag.getLocation();
        double angleToTarget = calculateTurnAngle(target_position);
        if (isInFront(angleToTarget)){
            double collisionProb = calculateCollisionProbability(position, target_position);
            if (collisionProb > 90){
                return true;
            }

        }
        return false;
    }

    public static double calculateTurnAngle(Point2D target){
        try{
            double yaw = Math.toDegrees(our_tag.getYaw());
            double angleToTarget = Math.toDegrees(Math.atan2(target.y - position.y,
                    target.x - position.x));

            // Normalize the angleToTarget to a 0-360 range
            angleToTarget = (angleToTarget + 360) % 360;

            // Calculate the difference
            double angleDifference = angleToTarget - yaw;

            // Normalize the angleDifference to a -180 to 180 range
            angleDifference = (angleDifference + 180) % 360 - 180;
            if (angleDifference > 180) {
                angleDifference -= 360;
            } else if (angleDifference < -180) {
                angleDifference += 360;
            }
            return angleDifference;
        }
        catch (Exception e){
            System.out.println(e);
        }


        return 0;
    }

    public static boolean isInFront(double angle1){
        angle1 = Math.abs(angle1);
        if (angle1 < 15 || angle1 > 160){
            return true;
        }
        return false;
    }

    public static double calculateCollisionProbability(Point2D position1, Point2D position2) {
        double distance = calcDistance(position2);
        distance /= 5;

        if (distance <= 15) {
            return 100; // 100% probability if distance is 25 or less
        } else if (distance <= 250) {
            // Linear scaling for distances between 25 and 250
            return 100 - ((distance - 25) * (100 - 1) / (250 - 25));
        } else {
            return 1; // 1% probability if distance is 2500 or more
        }

    }


    public static double calcDistance(Point2D target) {
            double xdiff = Math.abs(target.x - position.x);
            double ydiff = Math.abs(target.y - position.y);
            double result = Math.sqrt((ydiff * ydiff) + (xdiff * xdiff));
            return result;
    }
}