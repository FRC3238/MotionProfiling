/**
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * 
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.  
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral. 
 * 
 * While holding top-left-shoulder-button5, tap top-right-shoulder-button6.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * 
 * Release button5 to allow OpenVoltage control with left y axis.
 */

package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import org.usfirst.frc.team3238.robot.MotionProfileExample.PeriodicRunnable;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    /** The Talon we want to motion profile. */
    CANTalon _talon = new CANTalon(6), _talonA = new CANTalon(7),
            _talonB = new CANTalon(1), _talonC = new CANTalon(2);
//    ArrayList<Integer> leftCount = new ArrayList<Integer>(), rightCount = new ArrayList<Integer>();
    /** some example logic on how one can manage an MP */
    MotionProfileExample _example = new MotionProfileExample(_talon),
            _exampleA = new MotionProfileExample(_talonB);
    
    /** joystick for testing */
    Joystick _joy= new Joystick(0);

    /** cache last buttons so we can detect press events.  In a command-based project you can leverage the on-press event
     * but for this simple example, lets just do quick compares to prev-btn-states */
    boolean [] _btnsLast = {false,false,false,false,false,false,false,false,false,false};

    int _loops = 0;
    
    public Robot() { // could also use RobotInit()
//        _talon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
//        _talon.reverseSensor(false); /* keep sensor and motor in phase */
//        _talon.setInverted(false);
    }
    
    int _mpStatus = 0;
    
    class PeriodicRunnable implements java.lang.Runnable {
        public void run() 
        {  
            _talon.processMotionProfileBuffer();    
            _talonB.processMotionProfileBuffer();
        }
    }
    
    Notifier _notifer = new Notifier(new PeriodicRunnable());

    private CANTalon.MotionProfileStatus _status = new CANTalon.MotionProfileStatus();
    
    public void robotInit()
    {
        _talon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
//        _talon.setForwardSoftLimit(forwardLimit);
        _talon.configEncoderCodesPerRev(360);
        _talonB.configEncoderCodesPerRev(360);
        _talon.reverseSensor(false); /* keep sensor and motor in phase */
        _talon.setInverted(true);        
        _talonA.changeControlMode(TalonControlMode.Follower);
        _talonA.set(_talon.getDeviceID());
        _talonC.changeControlMode(TalonControlMode.Follower);
        _talonC.set(_talonB.getDeviceID());
    }
    
    double pVal = 0;
    public void autonomousInit() {
        Preferences preferences = Preferences.getInstance();
        pVal = preferences.getDouble("pVal", 0.0);
        _talon.setPosition(0);
        _talonB.setPosition(0);
        _talon.clearMotionProfileTrajectories();
        _talon.changeControlMode(TalonControlMode.MotionProfile);
        _talon.set(CANTalon.SetValueMotionProfile.Disable.value);
        _talon.changeMotionControlFramePeriod(5);
        //_talon.reverseSensor(true);
        _talonB.clearMotionProfileTrajectories();
        _talonB.changeControlMode(TalonControlMode.MotionProfile);
        _talonB.set(CANTalon.SetValueMotionProfile.Disable.value);
        _talonB.changeMotionControlFramePeriod(5);
        _talon.reverseOutput(true);
        _talonB.reverseSensor(true);
        _talon.setF(0.8103);
        _talon.setP(2.0);
        _talon.setI(0.0);
        _talon.setD(0.0);
        _talonB.setF(0.763);
        _talonB.setP(2.0);
        _talonB.setI(0.0);
        _talonB.setD(0.0);
        _mpStatus = 0;
        _loops = 0;
        complete = false;
        DriverStation.reportError(""+pVal, false);
    }
    
    /**  function is called periodically during operator control */
    boolean complete = false;
    public void autonomousPeriodic() {
        _talon.getMotionProfileStatus(_status); 
        _talonB.getMotionProfileStatus(_status);
        DriverStation.reportWarning("auto periodic: MP status = " + _status.toString(), false);
        switch (_mpStatus)
        {
            case 0:
                DriverStation.reportWarning("auto case 0", false);
                _notifer.startPeriodic(0.005);
                _example.startFilling();       
                _exampleA.startFilling();
                _talon.set(CANTalon.SetValueMotionProfile.Enable.value);
                _talonB.set(CANTalon.SetValueMotionProfile.Enable.value);
                _mpStatus++;
                break;
            default:
                DriverStation.reportWarning("auto case 1", false);
                //_talon.set(CANTalon.SetValueMotionProfile.Enable.value);
                if(!complete) {
                    
                double motorOutput = _talon.getOutputVoltage() / _talon.getBusVoltage();
                double altMotorOutput = _talonB.getOutputVoltage() / _talonB.getBusVoltage();
                System.out.println("motor output: " + motorOutput);
                System.out.println("motor speed: " + _talon.getSpeed());
                System.out.println("error in native units: " + _talon.getClosedLoopError());
                }

                //_loops++;
                if (_status.activePointValid && _status.activePoint.isLastPoint)
                {
                    SmartDashboard.putNumber("Loops", _loops);
                    DriverStation.reportWarning("auto case default: profile done", false);
                    _talon.set(CANTalon.SetValueMotionProfile.Disable.value);  
                    _talonB.set(CANTalon.SetValueMotionProfile.Disable.value);
                    DriverStation.reportWarning("# auto loops: " + _loops, false);
                    System.out.println(_talon.getPosition());
                    System.out.println(_talonB.getPosition() + " " + pVal);
                    System.out.println(pVal);
                    complete = true;
                }
                _loops++;
                
                break;
        }
    }

    Timer t = new Timer();
    public void teleopInit() {
        _talon.changeControlMode(TalonControlMode.PercentVbus);
        _talonB.changeControlMode(TalonControlMode.PercentVbus);
        _talon.setPosition(0);
        _talonB.setPosition(0);
        t.reset();
        t.start();
    }
    int leftCount = 0, rightCount = 0, counterA = 0;
    public void teleopPeriodic() {
        
       _talon.set(1.0);
       _talonB.set(1.0);
       if(t.get() > 0.1) {
//       System.out.println(t.get());
           leftCount += _talon.getPosition();
           rightCount += _talonB.getPosition();
           counterA++;
       System.out.println((leftCount/counterA) + " " + (rightCount/counterA));
       t.reset();
       _talon.setPosition(0);
       _talonB.setPosition(0);
       }

    }
    /**  function is called periodically during disable */
    public void disabledPeriodic() {
        /* it's generally a good idea to put motor controllers back
         * into a known state when robot is disabled.  That way when you
         * enable the robot doesn't just continue doing what it was doing before.
         * BUT if that's what the application/testing requires than modify this accordingly */
        _talon.changeControlMode(TalonControlMode.PercentVbus);
        _talon.set( 0 );
        _talonB.set(0);
        /* clear our buffer and put everything into a known state */
        _example.reset();
        _exampleA.reset();
    }
}