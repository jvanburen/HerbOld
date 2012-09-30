
import lejos.nxt.*;
import lejos.nxt.addon.GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.SegowayPilot;

/**
 *
 * @author Jacob
 */
public class HerbTwo {

    public static final boolean DEBUG = true;
    public static final int RIGHT = -1, LEFT = 1;
    public static final double WHEEL_CIRCUMFERENCE = 13.335; // cm
    public static final double WHEEL_DIAMETER = 5.6;//cm
    public static final int SERVO_STRENGTH = 900;
    public static final int MOVE_DELAY = 500;//ms
    public static final int DETECT_DISTANCE = 60;//cm
    public static final int REVERSE_DETECT_DISTANCE = 30;//cm
    public static final int ARC_SIZE = 60;//wheel differential in percent
    public static final int TRAVEL_SPEED = 100; //arbitrary, > 0
    public static final int TURN_DIRECTION = RIGHT; //RIGHT or LEFT
    //
    //sensors
    //
    private static final GyroSensor GYRO_SENSOR = new GyroSensor(SensorPort.S2);
    private static final UltrasonicSensor US_SENSOR = new UltrasonicSensor(SensorPort.S1);
    //
    //motors
    //
    private static final NXTRegulatedMotor LEFT_MOTOR = Motor.B;
    private static final NXTRegulatedMotor RIGHT_MOTOR = Motor.A;
    private static final EncoderMotor LEFT_ENCODER_MOTOR = new NXTMotor(MotorPort.B);
    private static final EncoderMotor RIGHT_ENCODER_MOTOR = new NXTMotor(MotorPort.A);
    //
    //static variables
    //
    private static SegowayPilot GYRO_CONTROLLER;
    private static NavigatorThread NAVIGATOR;

    /**
     * Steers herb away from obstacles
     */
    private static class NavigatorThread extends Thread implements Runnable,
            MoveListener {

        private boolean enabled = false;

        @Override
        public final void run() {
            int distance;
            while (true) {
                if (enabled) {
                    distance = US_SENSOR.getDistance();
                    if (distance < REVERSE_DETECT_DISTANCE) {
                        reverse();
                    } else if (US_SENSOR.getDistance() < DETECT_DISTANCE) {
                        GYRO_CONTROLLER.steer(ARC_SIZE,
                                TURN_DIRECTION * 90, true);
                    } else {
                        GYRO_CONTROLLER.backward();
                    }
                }

                sleepFor(100);
            }
        }

        private void reverse() {
            GYRO_CONTROLLER.setTravelSpeed(10);
            sleepFor(100);
            GYRO_CONTROLLER.forward();

            sleepFor(2000);//reverse for a bit
            GYRO_CONTROLLER.backward();
            sleepFor(750);//get back to normal
            GYRO_CONTROLLER.setTravelSpeed(TRAVEL_SPEED);
        }

        @Override
        public synchronized final void moveStarted(Move move, MoveProvider mp) {
            if (!this.isAlive()) {
                start();
            }
        }

        @Override
        public synchronized final void moveStopped(Move move, MoveProvider mp) {
        }

        public synchronized final void enable() {
            enabled = true;
        }

        public synchronized final void disable() {
            enabled = false;
        }
    }

    public static void main(String[] a) {

        try {
            debug("Initializing...");
            init();
            debug("Initialization Complete.");
        } catch (InterruptedException ex) {
            if (DEBUG) {
                String message = ex.getMessage();
                if (message != null) {
                    print(message);
                }
            }
        }

        try {
            debug("Running...");
            run();
            debug("Program Completed Successfully.");
        } catch (Exception ex) {
            if (DEBUG) {
                String message = ex.getMessage();
                if (message != null) {
                    print(message);
                }
            }
        }
        while (GYRO_CONTROLLER.isAlive()) {
            sleepFor(250);
        }

    }

    public synchronized static void init() throws InterruptedException {
        LEFT_MOTOR.setSpeed(SERVO_STRENGTH);
        RIGHT_MOTOR.setSpeed(SERVO_STRENGTH);

        GYRO_CONTROLLER = new SegowayPilot(
                LEFT_ENCODER_MOTOR,
                RIGHT_ENCODER_MOTOR,
                GYRO_SENSOR,
                WHEEL_DIAMETER,
                10e2);

        NAVIGATOR = new NavigatorThread();
        NAVIGATOR.disable();
        GYRO_CONTROLLER.addMoveListener(NAVIGATOR);
        debug("MoveListener attached");

    }

    public synchronized static void run() throws InterruptedException {
        sleepFor(5000);//let it balance
        GYRO_CONTROLLER.setTravelSpeed(TRAVEL_SPEED);
        GYRO_CONTROLLER.setMoveDelay(MOVE_DELAY);
        GYRO_CONTROLLER.backward();//actually forward

    }

    //<editor-fold defaultstate="collapsed" desc="Print and Debug Statements">
    public static void print(Object o) {
        System.out.println(o);
    }

    public static void print(Object o, Character ending) {
        if (ending == null) {
            System.out.print(o.toString());
        } else {
            System.out.print(o.toString() + ending);
        }
    }

    public static void debug(Object o) {
        if (!DEBUG) {
            return;
        }

        System.out.println(o);

    }

    public static void debug(Object o, Character ending) {
        if (!DEBUG) {
            return;
        }

        if (ending == null) {
            System.out.print(o.toString());
        } else {
            System.out.print(o.toString() + ending);
        }

    }

    public static void sleepFor(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ex) {
            System.out.println("Exception thrown in sleepFor");
            System.exit(1);
        }
    }
    //</editor-fold>
}
