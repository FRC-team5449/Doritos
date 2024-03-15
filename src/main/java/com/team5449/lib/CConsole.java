package com.team5449.lib;

import java.io.OutputStream;

/**
 * This Class contains a few useful fuctions about console.
 * 
 * @author Freeman Zhao
 * 
 * TODO: Check if these functions actually causes no heap pollution.
 */
public class CConsole{
    public final static class Color{
        static final byte[] Red="\033[1;31m".getBytes();
        static final byte[] Yellow="\033[1;33m".getBytes();
        static final byte[] Default="\033[0m".getBytes();
    }
    
    public final static CConsole stdout=new CConsole(System.out);
    public final static CConsole stderr=new CConsole(System.err);
    private final OutputStream out;
    public CConsole(){
        this.out = System.out;
    }
    public CConsole(OutputStream out){
        this.out = out;
    }
    @SuppressWarnings("unchecked")
    private <T> void ColoredPrint(byte[] c,T... Args)
    {
        write(c);
        print(Args);
        write(Color.Default);
    }
    /**
     * Print stack trace of current thread.
     * @param IgnoreLevel The numbers of top calling thread to be ignored, 0 to print java.base/java.lang.Thread.getStackTrace(Thread.java:1610)
     * @param MaximumPrints The maximum numbers of line PrintTraceBack can print, set to 0 to print all lines
     * @apiNote This fuction will raise a range error when IgnoreLevel<0, so make sure you set IgnoreLevel>=0.
     */
    public void PrintTraceBack(int IgnoreLevel, int MaximumPrints)
    {
        StackTraceElement[] st=Thread.currentThread().getStackTrace();
        byte[] Seperater=System.getProperty("line.separator","\n").getBytes();
        int upper;
        if(MaximumPrints!=0)
        {
            upper = Math.min(IgnoreLevel+MaximumPrints, st.length);
        }else upper = st.length;
        for(int i=IgnoreLevel;i<upper;i++)
        {
            PrintString(st[i].toString());
            write(Seperater);
        }
    }
    private void write(byte[] b)
    {
        try{
            out.write(b);
        }catch(java.io.IOException e)
        {
            stderr.error(e);
        }
    }
    private void write(int b)
    {
        try{
            out.write(b);
        }catch(java.io.IOException e)
        {
            stderr.error(e);
        }
    }
    private void PrintString(String s)
    {
        write(s.getBytes());
    }
    /**
     * Print some of the values to default console.
     * @param <T> Any type you want, you can even mix the types
     * @param Args
     * Note: Use <T> to reduce class convertion to Object and when Args contains String, char or char[], it not calls the .toString method.
     * @apiNote This function doesn't print new line when ending. Use {@link CConsole#log} instead.
     * @see https://docs.oracle.com/en/java/javase/21/docs/api/java.base/java/io/PrintStream.html#print(boolean)
     */
    @SuppressWarnings("unchecked")
    public <T> void print(T... Args)
    {
        if(Args==null)
        {
            PrintString("null");
            return;
        }
        for(int i=0;i<Args.length-1;i++)
        {
            PrintString(String.valueOf(Args[i]));
            write(' ');
        }
        PrintString(String.valueOf(Args[Args.length-1]));
    }
    /**
     * Print a red-colored error to default console.
     * @see CConsole#print
     */
    @SuppressWarnings("unchecked")
    public <T> void error(T... Args)
    {
        write(Color.Red);
        print(Args);
        PrintString(" at ");
        PrintTraceBack(3,0);
        write(Color.Default);
    }
    /**
     * Print a yellow-colored warn to default console.
     * @see CConsole#print
     */
    @SuppressWarnings("unchecked")
    public <T> void warn(T... Args)
    {
        ColoredPrint(Color.Yellow, Args);
    }
    /**
     * Print some values to console with where the function is called and a new line at end of input.
     * @param Args Values you want to print.
     * @apiNote This function is useful when you have multiple code printing at the same time.
     * It will be hard to identify what code print a specified line, but this function will print
     * where it was called, making it useful to remove/debug codes.
     * Example:
     * {@code log("1",2)} will print {@code "1 2 at FUNCTION(FILE:LINE)\n"}
     * @see JavaScript{@code console.log()} function.
     */
    @SuppressWarnings("unchecked")
    public <T> void log(T... Args)
    {
        print(Args);
        PrintString(" at ");
        PrintTraceBack(3,1);
        //write('\n');
    }
    @Override
    public String toString() {
        return "CConsole [OutputStream = " + out.getClass() + "]";
    }
}