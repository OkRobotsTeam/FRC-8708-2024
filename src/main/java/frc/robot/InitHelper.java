package frc.robot;

/**
 * Helper class for managing initialization processes.
 */
public class InitHelper {
    private long initStartTime;
    private final long minimumTime;
    private boolean initialized;
    private double lastSpeed;
    private final double minValueDifference;
    private final String name;
    private long lastInitCheckTime;
    private final long minTimeDelta;
    private boolean justFinishedInit = false;
    private final long failsafeTimeLimitMillis;

    /**
     * Constructs an InitHelper instance with the provided parameters.
     *
     * @param name               The name of the item being initialized (for debugging purposes).
     * @param minValueDifference Differences below this trigger initialization being finished.
     * @param minTimeMillis      The minimum time to initialize for. Differences below the trigger will be ignored during this time.
     * @param maxTimeMillis      The limit for how long the initialization process can take, providing a failsafe timeout.
     * @param minTimeDelta       The minimum time between checking for the trigger. Calls made too soon after the last one will be skipped.
     */
    public InitHelper(String name, double minValueDifference, long minTimeMillis, long maxTimeMillis, long minTimeDelta) {
        this.name = name;
        this.minimumTime = minTimeMillis;
        this.minValueDifference = minValueDifference;
        this.failsafeTimeLimitMillis = maxTimeMillis;
        this.minTimeDelta = minTimeDelta;
    }

    /**
     * Starts the initialization process with the given initial speed value.
     *
     * @param initialValue The initial value for the initialization process.
     */
    public void start(double initialValue) {
        initialized = false;
        initStartTime = System.currentTimeMillis();
        lastSpeed = initialValue;
    }

    /**
     * Checks if the initialization process is still ongoing.
     *
     * @return true if the initialization is still ongoing, false otherwise.
     */
    public boolean isInitializing() {
        return !initialized;
    }

    /**
     * Checks if the initialization process has been completed.
     *
     * @return true if the initialization process has been completed, false otherwise.
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Manually Marks the initialization process as done.
     */
    public void setDone() {
        debugOut(name + " manually set to initialized: setDone called");
        initialized = true;
        this.justFinishedInit = true;
    }

    /**
     * Checks if the initialization process is still ongoing with the given value.
     *
     * @param value The current value to check for initialization.
     * @return true if the initialization is still ongoing, false otherwise.
     */
    public boolean isInitializing(double value) {
        if (initialized) {
            return false;
        }
        update(value);
        return !initialized;
    }

    /**
     * Updates the initialization process with the new speed currentSpeed.
     *
     * @param currentSpeed The new currentSpeed to update the initialization process.
     */
    public void update(double currentSpeed) {
        long now = System.currentTimeMillis();
        long timeSinceInitStart = now - initStartTime; // Calculate the time elapsed since initialization started
        long timeSinceLastCheck = now - lastInitCheckTime; // Calculate the time elapsed since the last update check

        if (timeSinceInitStart > failsafeTimeLimitMillis) {
            // Handle timeout: If the initialization process exceeds the maximum allowed time, terminate the process
            debugOut("Timeout reached. Giving up on init for " + name);
            initialized = true; // Mark initialization as complete
            this.justFinishedInit = true; // Set the flag indicating that initialization just finished
            return;
        }

        if (timeSinceLastCheck < minTimeDelta) {
            // Return immediately if this call to update was made too soon after the last one
            return;
        }

        lastInitCheckTime = now; // Update the last update check time

        if (timeSinceInitStart < minimumTime) {
            // If the minimum initialization time has not elapsed yet, continue initialization without checking for speed differences
            debugOut(name + " is still initializing " + (currentSpeed - lastSpeed));
            lastSpeed = currentSpeed; // Update the last speed for future comparisons
            return;
        }

        if (Math.abs(currentSpeed - lastSpeed) <= minValueDifference) {
            // If the speed difference is within the acceptable range, mark initialization as complete
            debugOut(name + " done init " + (currentSpeed - lastSpeed) + " <= " + minValueDifference);
            initialized = true;
            this.justFinishedInit = true; // Set the flag indicating that initialization just finished
        } else {
            // If the speed difference is still significant, continue initialization
            debugOut(name + " still moving " + (currentSpeed - lastSpeed) + " <= " + minValueDifference);
        }

        lastSpeed = currentSpeed; // Update the last speed for future comparisons
    }

    /**
     * Checks if the initialization process has just been finished.
     *
     * @return true if the initialization process has just been finished, false otherwise.
     */
    public boolean justFinishedInit() {
        if (justFinishedInit) {
            justFinishedInit = false;
            return true;
        }
        return false;
    }

    private void debugOut(String line) {
        System.out.println(line);
    }
}
