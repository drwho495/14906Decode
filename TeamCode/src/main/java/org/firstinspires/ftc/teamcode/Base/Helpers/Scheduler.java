package org.firstinspires.ftc.teamcode.Base.Helpers;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class Scheduler {
    public HashMap<Double, Runnable> scheduleMap = new HashMap<Double, Runnable>();
    public HashMap<Double, Runnable> runScheduleMap = new HashMap<Double, Runnable>();
    private boolean hasRun = false;
    private boolean isFinished = false;
    private TimeUnit timerUnit = TimeUnit.MILLISECONDS;
    private ElapsedTime timer = new ElapsedTime();

    public int getNumberOfSchedules() {
        return scheduleMap.size();
    }

    public void setTimeUnit(TimeUnit newTimeUnit) {
        timerUnit = newTimeUnit;
    }

    public void reset(boolean clearSchedules) {
        hasRun = false;
        isFinished = false;
        runScheduleMap.clear();

        if (clearSchedules) {
            scheduleMap.clear();
        }

        timer.reset();
    }

    public void cancel() {
        isFinished = true;
        hasRun = true;
        runScheduleMap.clear();
    }

    public Scheduler addMethod(double time, Runnable runnable) {
        this.scheduleMap.put(time, runnable);

        return this;
    }

    public boolean finished() {
        return isFinished;
    }

    public void update() {
        if (!isFinished) {
            if (!hasRun) {
                runScheduleMap = scheduleMap;

                timer.reset();
                hasRun = true;
                isFinished = false;
            }
        } else {
            return;
        }

        for (Map.Entry<Double, Runnable> item: ((HashMap<Double, Runnable>) runScheduleMap.clone()).entrySet()) {
            if (item.getKey() <= timer.time(timerUnit)) {
                item.getValue().run();
                runScheduleMap.remove(item.getKey());
            }
        }

        if (runScheduleMap.isEmpty()) {
            isFinished = true;
        }
    }
}
