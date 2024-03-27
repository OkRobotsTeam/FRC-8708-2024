package frc.robot;

public class InitHelper {
    private long initStartTime;
    private long minimumTime;
    private boolean initialized; 
    private double lastValue;
    private double minValueDifference;
    private String name;
    private long lastCheckTime;
    private long minTimeDelta;
    private boolean justFinishedInit = false;
    private long maxTimeMillis;

/*
 * @param name The name of what is being inited for debug output
 * @param minValueDifference Differences below this trigger init being finished
 * @param minTimeMillis The minimum time to init for.  Differences below the trigger will be ignored during this time
 * @param maxTimeMillis The limit for how long the init process can take.  Provides a failsafe timeout.
 * @param minTimeDelta  The minimum time between checking for trigger.  Calls too often will be skipped.
 */
   
    public InitHelper(String name, double minValueDifference, long minTimeMillis, long maxTimeMillis ) {
        this(name, minValueDifference, minTimeMillis,  maxTimeMillis, 20);
    }

    public InitHelper(String name, double minValueDifference, long minimumTimeMillis, long maxTimeMillis, long minTimeDelta) {
        this.name = name;
        this.minimumTime = minimumTimeMillis;
        this.minValueDifference = minValueDifference;
        this.maxTimeMillis = maxTimeMillis;
        this.minTimeDelta = minTimeDelta;
    }
    public void start(double initialValue) {
        
        initialized = false ;
        initStartTime = System.currentTimeMillis();
        lastValue = initialValue;
    }

    public boolean initializing() {
        return !initialized;
    }
    public boolean initialized() {
        return initialized;
    }

    public void setDone() {
        debugOut(name + " done init: setDone called");
        initialized = true;
        this.justFinishedInit = true;
    }

    public boolean isInitializing(double value) {
        if (initialized) { 
            return false;
        }
        doneIfUnchanged(value);
        return !initialized;
    }

    public void doneIfUnchanged(double value) {
        long now = System.currentTimeMillis();
        if ((now - initStartTime) < minimumTime) {
            debugOut(name + " still in timeout " + (value - lastValue));
            lastValue = value;
            lastCheckTime  = now;
            return;
        }
        if ( (now - lastCheckTime) < minTimeDelta) {
            //debugOut("Called multiple times within " + (now - lastCheckTime) + " milliseconds.  Aborting check.");
            return;
        }
        if ( (now - initStartTime) > maxTimeMillis) {
            debugOut("Timeout reached.  Giving up on init for "+ name);
            initialized = true;
            this.justFinishedInit = true;
            return;
        }

        lastCheckTime = now;
        if (minValueDifference>=0) {
            if ((value - lastValue) <= minValueDifference) {
                debugOut(name + " done init " + (value - lastValue) + " <= " + minValueDifference);
                initialized = true;
                this.justFinishedInit = true;
            } else {
                debugOut(name + " still moving " + (value - lastValue) + " <= " + minValueDifference);
            }
        } else {
            if ((value - lastValue) > minValueDifference) {
                debugOut(name + " done init " + (value - lastValue) + " > " + minValueDifference);
                initialized = true;
                this.justFinishedInit = true;
            } else {
                debugOut(name + " still moving " + (value - lastValue) + " > " + minValueDifference);
            }
        }
        lastValue = value;
    }


    public boolean justFinishedInit() {
        if (justFinishedInit == true) {
            justFinishedInit=false;
            return true;
        } else {
            return false;
        }
    }
    private void onInit() {
        //onInit.run();;
    }

    private void debugOut(String line) {
        //Comment out to suppress
        //System.out.println("InitHelper " + System.currentTimeMillis() + ": " + line);
        System.out.println(line);
        
    }

    public void start(double position, Object object) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'start'");
    }

}