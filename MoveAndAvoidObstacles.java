import lejos.nxt.*;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.Color;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

/**
 * Created by sa16566 on 26/01/2018.
 */
public class MoveAndAvoidObstacles {
    public static void main(String[] args) {
        //moveThread.run(); ...........
        moveClass.run();
    }
    public static class moveClass {
        public static void run() {
            DifferentialPilot pilot = new DifferentialPilot(3.4, 21, Motor.A, Motor.C, false);
            OdometryPoseProvider opp = new OdometryPoseProvider (pilot) ;
            Navigator nav = new Navigator(pilot, opp);
            try
            {
                Behavior[] bArray;
                bArray = new Behavior[]{new moveRobot(pilot, nav, opp), new search(pilot, nav), new foundColour(SensorPort.S2, nav),  new avoidObstacle(opp, SensorPort.S3, pilot, nav)};
                Arbitrator arb = new Arbitrator(bArray);
                nav.addWaypoint(150, 150);
                nav.singleStep(true);
                arb.start();
                nav.stop();
            } catch (
                    Exception e)
            {
                System.out.println("Exception");
            }
        }

        public static class search implements Behavior {

            private Navigator nav;
            private DifferentialPilot pilot;
            private boolean suppressed = false;

            public search(DifferentialPilot pilot, Navigator nav) {
                this.pilot = pilot;
                this.nav = nav;
            };

            @Override
            public void action() {
                pilot.setRotateSpeed(90);
                pilot.setTravelSpeed(30);
                suppressed = false;
                while(!suppressed){
                    pilot.rotate(30);
                    if(suppressed){break;};
                    pilot.travel(10);
                    if(suppressed){break;};
                    pilot.travel(10);
                    if(suppressed){break;};
                    pilot.travel(10);
                    if(suppressed){break;};
                    pilot.travel(-10);
                    if(suppressed){break;};
                };
            };

            @Override
            public void suppress() {
                suppressed = true;
            };

            @Override
            public boolean takeControl() {
                return nav.pathCompleted();
            };
        };

        public static class foundColour implements Behavior {

            private Navigator nav;
            private ColorSensor sensor;
            private boolean suppressed = false;

            public foundColour(SensorPort port, Navigator nav) {
                this.sensor = new ColorSensor(port);
                this.nav = nav;
            };

            @Override
            public void action() {
                suppressed = false;
                if(!suppressed){
                    nav.addWaypoint(0, 0);
                    nav.followPath();
                };
            };

            @Override
            public void suppress() {
                suppressed = true;
            };

            @Override
            public boolean takeControl() {
                return (sensor.getColorID() == ColorSensor.Color.RED);
            };
        };

        public static class moveRobot implements Behavior{
            private Navigator nav;
            private DifferentialPilot pilot;
            private OdometryPoseProvider opp;
            private boolean suppressed = false;
            public moveRobot(DifferentialPilot pilot, Navigator nav, OdometryPoseProvider opp) {
                this.pilot = pilot;
                this.nav = nav;
                this.opp = opp;
            };
            @Override
            public void action() {
                suppressed = false;
                pilot.setRotateSpeed(90);
                pilot.setTravelSpeed(30);
                while(!suppressed){
                    nav.followPath();
                    LCD.setPixel((10+((int)opp.getPose().getX()/3)), 10+((int)opp.getPose().getY()/3),1);
                };
                /*
                if (suppressed){
                    Thread.yield();
                    pilot.stop();
                }
                */

            };

            @Override
            public void suppress() {
                suppressed = true;
            };
            @Override
            public boolean takeControl() {
                return true;
            };
        }


        public static class avoidObstacle implements Behavior{
            private Navigator nav;
            private DifferentialPilot pilot;
            private UltrasonicSensor sensor;
            private OdometryPoseProvider opp;
            private boolean suppressed = false;
            public avoidObstacle(OdometryPoseProvider opp, SensorPort port, DifferentialPilot pilot, Navigator nav) {
                this.sensor = new UltrasonicSensor(port);
                this.pilot = pilot;
                this.nav = nav;
                this.opp = opp;
            };
            @Override
            public void action() {
                suppressed = false;
                while(!suppressed){
                    nav.stop();
                    //System.out.println("Avoiding Obstacle");
                    pilot.rotate(80);
                    LCD.setPixel((10+((int)opp.getPose().getX()/3)), 10+((int)opp.getPose().getY()/3),1);
                    pilot.travel(30);
                    LCD.setPixel((10+((int)opp.getPose().getX()/3)), 10+((int)opp.getPose().getY()/3),1);
                    pilot.rotate(-80);
                    LCD.setPixel((10+((int)opp.getPose().getX()/3)), 10+((int)opp.getPose().getY()/3),1);
                    pilot.travel(30);
                    LCD.setPixel((10+((int)opp.getPose().getX()/3)), 10+((int)opp.getPose().getY()/3),1);
                    break;
                };
            };

            @Override
            public void suppress() {
                suppressed = true;
            };

            @Override
            public boolean takeControl() {
                return (sensor.getDistance() <= 20);
            };
        }
    }
}

//pilot.steer(-50, 180, true); // turn 180 degrees to the right
//pilot.steer(100);          // turns with left wheel stationary
//while (pilot.isMoving()) Thread.yield();
