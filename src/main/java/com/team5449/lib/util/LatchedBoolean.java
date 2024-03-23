package com.team5449.lib.util;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBoolean {
    private boolean mLast = false;

    public boolean update(boolean newValue) {
        boolean ret = false;
        if (newValue && !mLast) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}
// Copied from team 254's 2019 code. https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/util/LatchedBoolean.java