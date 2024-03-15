package com.team5449.lib;

import java.util.ArrayList;
import java.util.List;

/**
 * A simlpe class that packed the ordinary setter and getter.
 * Pitty that Java can't reload operator like =, so we can't make
 * the code immersive and have more capiable to change.
 * @param <T> Any type you want to declear, but not basic data type.
 * @example
 * <pre> 
 * Changeable<Double> a = new Changeable<Double>("a",0.0);
 * a.getValue();//Equivalent to a
 * a.setValue(10.0);//a=10.0
 * a.setValue(10.000);//Do nothing
 * a.getValue();//Returns 10.0
 * </pre>
 * 
 * @author Freeman Zhao
 * @apiNote Like = operator, This Changeable class DOES NOT make deep copy of {@code <T>}
 */
public class Changeable<T extends Comparable<T>> {
    protected T mVal;
    private final String mName;
    private List<Runnable> mOnValueChange = new ArrayList<>();
    protected Runnable InternalonValueChange = null;
    private final Object mLock = new Object();// Use mLock to make Thread lock, because syncorized(..getValue()) might occour.
    private final Object mNotify = new Object();// notify() when values are updated.
    /*public Changeable(T val)
    {
        mVal = val;
        mName = val.getClass().getSimpleName()+" "+val.toString();
    }*/
    /**
     * Initalized the {@code Changeable} class
     * @param Name The variable name you want (Recommend to make it same with the actual variable name)
     * @param initVal The inital value of the variable
     */
    public Changeable(String Name,T initVal)
    {
        mName = Name;
        mVal = initVal;
    }
    /**
     * Construct the {@code Changeable} class with another instance (Copy all the stuff)
     * @param mInstance The instance you want to construct.
     */
    public Changeable(Changeable<T> mInstance)
    {
        this.mVal = mInstance.mVal;
        this.mName = mInstance.mName;
        this.mOnValueChange = mInstance.mOnValueChange;
        this.InternalonValueChange = mInstance.InternalonValueChange;
    }
    /**
     * Get the current value of the variable
     * @return The vaule
     */
    public T getValue(){
        //CConsole.stdout.error(mVal);
        synchronized(mLock){
            return mVal;
        }
    }
    /**
     * Set the vaule to {@code var}
     * @param var The new vaule you want to set.
     * @apiNote This function will call {@code OnValueChange} once you set a different value from 
     * the one in the class (changed value). You can add the {@code OnValueChange} function by calling {@link #setOnValueChange}
     */
    public void setValue(T var){
        if(!mVal.equals(var))
        {
            synchronized(mLock){
                mVal = var;
                CConsole.stdout.log("Setted value "+mVal);
            }
            mNotify.notifyAll();
            if(mOnValueChange!=null)
            {
                mOnValueChange.forEach((e)->{e.run();});
            }
            if(InternalonValueChange!=null)
            {
                InternalonValueChange.run();
            }
        }
    }
    /**
     * Set the value by internal, doesn't call the {@code InternalonValueChange} function
     * @see setValue
     */
    protected void InternalSetValue(T var)
    {
        if(!mVal.equals(var))
        {
            synchronized(mLock){
                mVal = var;
                CConsole.stdout.log("INTERNAL Setted value "+mVal);
            }
            mNotify.notifyAll();
            if(mOnValueChange!=null)
            {
                mOnValueChange.forEach((e)->{e.run();});
            }
        }
    }
    /**
     * Gets the variable name specified in initalize.
     * @return Variable name
     */
    public String getVaribleName()
    {
        return mName;
    }
    /**
     * Adds {@code OnValueChange} function 
     * @param mRunnable The function you want to called once the value has been changed.
     * @apiNote 
     * <p>The {@code mRunnable} function might be called in different thread depending 
     * on what thread calls it. So please using {@code syncorized}, or use 
     * {@link #waitforNextValueChange} instead.</p>
     * 
     * Also, to get rid of calling a function mutiple times, we checked if
     *  {@code mRunnable} is already in OnValueChange functions by comparing its
     * POINTER. But it can still be posible to have same code function of different pointer.
     * So make sure you don't miscall it.
     */
    public void setOnValueChange(Runnable mRunnable)
    {
        if(!mOnValueChange.contains(mRunnable))
        {
            mOnValueChange.add(mRunnable);
        }else{
            CConsole.stdout.error("Already added "+mRunnable);
        }
    }

    /**
     * Wait until the {@code value} is updated.
     * @param timeoutMs The timeout in millisecond.
     * @param ns The nanosecond part. (1 nanosecond = 10⁻⁹ s)
     * @return True if there's change in value
     * @apiNote Set {@code timeoutMs} and {@code ns} to zero to wait infinitely.
     * <p> There's possibility {@code waitforNextValueChange(0,0)} returns false
     *  because other thread can call {@link Thread#interrupt} to force it exits.</p>
     */
    public boolean waitforNextValueChange(long timeoutMs,int ns){
        try{
            mNotify.wait(timeoutMs, ns);
        }catch(Exception e){
            CConsole.stdout.log("mNotify.wait doesn't exit correctly.",e);
            return false;
        }
        return true;
    }
    /**
     * {@code TYPE VARIABLE_NAME = VALUE.toString()}
     * Example: {@code Double mTest = 10.0}
     */
    @Override
    public String toString()
    {
        synchronized(mLock){
            return mVal.getClass().getSimpleName()+" "+mName+" = "+mVal.toString();
        }
    }
}