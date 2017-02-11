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
    CANTalon _talon = new CANTalon(3), _talonA = new CANTalon(4),
            _talonB = new CANTalon(1), _talonC = new CANTalon(2);
//    ArrayList<Integer> leftCount = new ArrayList<Integer>(), rightCount = new ArrayList<Integer>();
    /** some example logic on how one can manage an MP */
    MotionProfileExample _example = new MotionProfileExample(_talon),
            _exampleA = new MotionProfileExample(_talonB);
    
    /** joystick for testing */
    Joystick _joy= new Joystick(0);

    /** cache last buttons so we can detect press events.  In a command-based project you can leverage the on-press event
     * but for this simple example, lets just do quick compares to prev-btn-states */

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
        _talon.reverseSensor(true); /* keep sensor and motor in phase */
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
        _talonB.reverseSensor(false);
        _talonB.reverseOutput(false);
        _talon.setF(0.76);
        _talon.setP(0.09);
//        _talon.setP(0);
//        _talonB.setP(0);
        _talon.setI(0.0);
        _talonB.setP(0.09);
        _talon.setD(0.0);
        _talonB.setF(0.76);
//        _talonB.setP(2.0);
        _talonB.setI(0.0);
        _talonB.setD(0.0);
//        _talon.configEncoderCodesPerRev(1440);
//        _talonB.configEncoderCodesPerRev(1440);
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
                _example.fillRight();       
                _exampleA.fillLeft();
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
    Joystick mainDrive = new Joystick(1);
    public void teleopPeriodic() {
        double twist = 0, y = 0;
        if(Math.abs(mainDrive.getTwist()) > 0.15){
           twist = mainDrive.getTwist()*.5;
        
        }
        if(Math.abs(mainDrive.getY()) > 0.15) {
        y = mainDrive.getY();
        } 
        _talon.set(y-twist);
        _talonB.set(y+twist);
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
        DriverStation.reportWarning("Talon 0: " + _talon.getEncPosition(), false);
        DriverStation.reportWarning("Talon B: " + _talonB.getEncPosition(), false);
        _talon.changeControlMode(TalonControlMode.PercentVbus);
        _talon.set( 0 );
        _talonB.set(0);
        /* clear our buffer and put everything into a known state */
        _example.reset();
        _exampleA.reset();
    }
}