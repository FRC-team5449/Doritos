package com.team5449.lib;

import java.util.EnumSet;

//import java.util.Timer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * A class that makes ShuffleBoard easy to adjust value of any type.
 * TODO: verify if the auto update function works correctly.
 * @author Freeman Zhao
 */
public class ShuffleBoardAdjustValue<T extends Comparable<T>> extends Changeable<T>{
    private GenericEntry mEntry;
    private static final NetworkTableInstance mInst = NetworkTableInstance.getDefault();
    private int mListenerId = -1;
    //private final IChangeable<T> mChangeable;
    //private final Timer timer = new Timer();
    private final String mtabName;
    private static final String mClassName; // Gets the class name (ShuffleBoardAdjustValue)
    // Reference: https://blog.csdn.net/lipei1220/article/details/9131325 方法3
    static{
        mClassName = new Object(){
            public String getClassName()
            {
                String clazzName = this.getClass().getName();
                return clazzName.substring(0, clazzName.lastIndexOf('$'));
            }
        }.getClassName();
    }
    //private Object mlastchange = null;

    /**
     * Gets the class of the caller function, then get its simple name(without package path)
     * @param ids The offset to desired caller function, 0 to get Thread.getStackTrace
     * @return The Class name
     */
    private static String GetCallerClassName(int ids)
    {
        String tmp = Thread.currentThread().getStackTrace()[ids].getClassName();//DO NOT USE .getClass().getName() or you will get Thread
        return tmp.substring(tmp.lastIndexOf('.')+1);
    }

    /**
     * Constructs a new instance
     * @param tabName The shuffuleboard tab name(NOT TITLE) you want, shown in top
     * @param mChangeable The instance of variable.
     */
    public ShuffleBoardAdjustValue(String tabName, Changeable<T> mChangeable)
    {
        super(mChangeable);
        CConsole.stdout.log(getVaribleName()+": "+getValue());
        mtabName = tabName;
        try {
            mEntry = Shuffleboard.getTab(tabName).add(getVaribleName(), getValue()).getEntry();
            
            InternalonValueChange = () -> {mEntry.setValue(getValue());};
            
            addListener();
        } catch (IllegalArgumentException e) {
            CConsole.stdout.error(e);
        }
        // this.mChangeable = mChangeable;
    }

    /**
     * Constructs using Changeable
     * @param mChangeable The value.
     * @apiNote This function will automatically set tab name to the caller's class name
     * @see {@link #ShuffleBoardAdjustValue}
     */
    public ShuffleBoardAdjustValue(Changeable<T> mChangeable){
        this(GetCallerClassName(3), mChangeable);
    }
    
    /**
     * Constructs with new Changeable()
     * @apiNote Equivalent to {@code ShuffleBoardAdjustValue<T>(new Changeable<T>(mName, initValue))}
     * This function will automatically set tab name to the caller's class name
     * @see {@link Changeable#Changeable} {@link ShuffleBoardAdjustValue#ShuffleBoardAdjustValue}
     */
    public ShuffleBoardAdjustValue(String mName,T initValue){
        this(GetCallerClassName(3), new Changeable<T>(mName,initValue));
        //CConsole.stdout.log("STACK TRACE", Thread.currentThread().getStackTrace()[2].getClassName());
    }

    /**
     * Check for any updates.
     * It should be called periodically in somewhere.
     */
    public void periodic(){
        //CConsole.stdout.log("Getting",mEntry.get().getValue(), "with inner value",getValue(),":",getValue().equals(mEntry.get().getValue()));
        if(getValue().equals(mEntry.get().getValue())==false)
        {
            //System.out.println(mEntry.getDouble(0));
            InternalSetValue((T)mEntry.get().getValue());
        }
    }

    /**
     * Unlistened the value change on ShuffleBoard, causing it won't modify when changes on ShuffleBoard are made.
     */
    public void removeListener(){
        if(mListenerId !=-1)
        {
            mInst.removeListener(mListenerId);
            mListenerId = -1;
        }
    }

    /**
     * Listened the value change on thread, will cause automitally update values. It will be automatically called in constructing.
     */
    public void addListener()
    {
        if(mListenerId == -1)
        {
            // See: https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-change.html#listening-for-changes
            mListenerId = mInst.addListener(mEntry, EnumSet.of(Kind.kValueRemote,Kind.kImmediate), event ->{
                CConsole.stdout.log("Caught event",event,"( value =", event.valueData.value.getValue());
                InternalSetValue((T)event.valueData.value.getValue());
            });
        }else{
            CConsole.stdout.error("Already added the event listener");
        }
    }

    /*@Override
    public T getValue()
    {
        if(mEntry!=null)
        {
            mVal = (T)mEntry.get().getValue();
        }
        System.out.println(mVal);
        return mVal;
    }*/
    //TODO: check if this file work correctly.
    @Override
    public String toString()
    {
        return mClassName+"("+mtabName+"/"+super.toString()+")";
    }
}