package frc.robot;

public class InitHelper {
    private long initStartTime;
    private long minimumTime;
    private boolean initialized; 
    private double lastValue;
    private double minDifference;
    private String name;
    private long lastCheckTime;
    private long minTimeDelta;
    private boolean justFinishedInit = false;

    public InitHelper(String name, long minimumTimeMillis, double minDifference) {
        this(name, minimumTimeMillis, minDifference, 20);
    }

    public InitHelper(String name, long minimumTimeMillis, double minDifference, long minTimeDelta) {
        this.name = name;
        this.minimumTime = minimumTimeMillis;
        this.minDifference = minDifference;
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

    public boolean initializing(double value) {
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
        lastCheckTime = now;
        if (minDifference>=0) {
            if ((value - lastValue) <= minDifference) {
                debugOut(name + " done init " + (value - lastValue) + " <= " + minDifference);
                initialized = true;
                this.justFinishedInit = true;
            } else {
                debugOut(name + " still moving " + (value - lastValue) + " <= " + minDifference);
            }
        } else {
            if ((value - lastValue) > minDifference) {
                debugOut(name + " done init " + (value - lastValue) + " > " + minDifference);
                initialized = true;
                this.justFinishedInit = true;
            } else {
                debugOut(name + " still moving " + (value - lastValue) + " > " + minDifference);
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
