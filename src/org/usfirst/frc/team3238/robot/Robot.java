package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import org.usfirst.frc.team3238.robot.MotionProfileExample.PeriodicRunnable;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import ProfileGenerator.Generator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{

    /** The Talon we want to motion profile. */
    CANTalon leftLeader = new CANTalon(3), leftFollower = new CANTalon(4),
            rightLeader = new CANTalon(1), rightFollower = new CANTalon(2);
    MotionProfileExample leftController = new MotionProfileExample(leftLeader),
            rightController = new MotionProfileExample(rightLeader);
    Generator g;

    Timer t = new Timer();
    boolean complete = false;

    int leftCount = 0, rightCount = 0, counterA = 0, controlStatus = 0,
            controlCalls = 0;

    double[][] leftProf, rightProf, mainProf;
    private CANTalon.MotionProfileStatus _status = new CANTalon.MotionProfileStatus();
    Joystick mainDrive = new Joystick(1);

    public Robot()
    {
    }

    class PeriodicRunnable implements java.lang.Runnable
    {
        public void run()
        {
            leftLeader.processMotionProfileBuffer();
            rightLeader.processMotionProfileBuffer();
        }
    }

    Notifier _notifer = new Notifier(new PeriodicRunnable());

    public void robotInit()
    {
        declareEncodersExist();
        setEncoderInversions();
        setFollowerTalons();
        setLiftProfilesLocally();
    }

    public void declareEncodersExist()
    {
        leftLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftLeader.configEncoderCodesPerRev(1440);
        rightLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightLeader.configEncoderCodesPerRev(1440);
    }

    public void setLiftProfilesLocally()
    {
        g = new Generator();
        g.generateSideProfile();
        leftProf = g.getLeftSideLift();
        rightProf = g.getRightSideLift();
        mainProf = g.getMainLift();
    }

    public void startMotionProfilesLoop()
    {
        pollProfileStatus();
        DriverStation.reportWarning(
                "auto periodic: MP status = " + _status.toString(), false);
        switch(controlStatus)
        {
            case 0:
                DriverStation.reportWarning("auto case 0", false);
                _notifer.startPeriodic(0.005);
                leftController.fillFed();
                rightController.fillFed();
                leftLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                rightLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                controlStatus++;
                break;
            default:
                DriverStation.reportWarning("auto case 1", false);
                // _talon.set(CANTalon.SetValueMotionProfile.Enable.value);
                if(!complete)
                {

                    double motorOutput = leftLeader.getOutputVoltage()
                            / leftLeader.getBusVoltage();
                    double altMotorOutput = rightLeader.getOutputVoltage()
                            / rightLeader.getBusVoltage();
                    System.out.println("motor output: " + motorOutput);
                    System.out.println("motor speed: " + leftLeader.getSpeed());
                    System.out.println("error in native units: "
                            + leftLeader.getClosedLoopError());
                }

                // _loops++;
                if(_status.activePointValid && _status.activePoint.isLastPoint)
                {
                    SmartDashboard.putNumber("Loops", controlCalls);
                    DriverStation.reportWarning(
                            "auto case default: profile done", false);
                    leftLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    rightLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    DriverStation.reportWarning("# auto loops: " + controlCalls,
                            false);
                    System.out.println(leftLeader.getPosition());
                    System.out.println(rightLeader.getPosition());
                    complete = true;
                }
                controlCalls++;

                break;
        }
    }

    public void prepEncodersForProfile()
    {
        resetEncoderPosition();
        enableMotionProfileMode();
        setEncoderInversions();
        setEncoderPID();
        resetDiagnostics();
    }

    public void disableMotors()
    {
        leftLeader.changeControlMode(TalonControlMode.PercentVbus);
        rightLeader.changeControlMode(TalonControlMode.PercentVbus);
        leftLeader.set(0);
        rightLeader.set(0);
        leftController.reset();
        rightController.reset();
    }

    public void setFollowerTalons()
    {
        leftFollower.changeControlMode(TalonControlMode.Follower);
        leftFollower.set(leftLeader.getDeviceID());
        rightFollower.changeControlMode(TalonControlMode.Follower);
        rightFollower.set(rightLeader.getDeviceID());
    }

    public void setEncoderPID()
    {
        leftLeader.setF(0.8);
        leftLeader.setP(0.0);
        leftLeader.setI(0.0);
        rightLeader.setP(0.0);
        leftLeader.setD(0.0);
        rightLeader.setF(0.8);
        rightLeader.setI(0.0);
        rightLeader.setD(0.0);
    }

    public void resetEncoderPosition()
    {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
    }

    public void setEncoderInversions()
    {
        leftLeader.reverseOutput(true);
        rightLeader.reverseSensor(false);
        rightLeader.reverseOutput(false);
    }

    public void enableMotionProfileMode()
    {
        leftLeader.clearMotionProfileTrajectories();
        leftLeader.changeControlMode(TalonControlMode.MotionProfile);
        leftLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        leftLeader.changeMotionControlFramePeriod(5);
        rightLeader.clearMotionProfileTrajectories();
        rightLeader.changeControlMode(TalonControlMode.MotionProfile);
        rightLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        rightLeader.changeMotionControlFramePeriod(5);

    }

    public void pollProfileStatus()
    {
        leftLeader.getMotionProfileStatus(_status);
        rightLeader.getMotionProfileStatus(_status);
    }

    public void resetDiagnostics()
    {
        controlStatus = 0;
        controlCalls = 0;
        complete = false;
    }

    public void setSideProfile()
    {
        leftController.setFedProfile(leftProf);
        rightController.setFedProfile(rightProf);
    }

    public void setStraightProfile()
    {
        leftController.setFedProfile(mainProf);
        rightController.setFedProfile(mainProf);
    }

    public void setMotorsDriveMode()
    {
        leftLeader.changeControlMode(TalonControlMode.PercentVbus);
        rightLeader.changeControlMode(TalonControlMode.PercentVbus);
    }

    public void resetTimer()
    {
        t.reset();
        t.start();
    }

    public void joystickControl()
    {
        double twist = 0, y = 0;
        if(Math.abs(mainDrive.getTwist()) > 0.15)
        {
            twist = mainDrive.getTwist() * .5;

        }
        if(Math.abs(mainDrive.getY()) > 0.15)
        {
            y = mainDrive.getY();
        }
        leftLeader.set(-y + twist);
        rightLeader.set(y + twist);
    }

    public void diagnostics() {
        if(t.get() > 0.1)
        {
            // System.out.println(t.get());
            leftCount += leftLeader.getPosition();
            rightCount += rightLeader.getPosition();
            counterA++;
            System.out.println(
                    (leftCount / counterA) + " " + (rightCount / counterA));
            t.reset();
            leftLeader.setPosition(0);
            rightLeader.setPosition(0);
        }
    }
    
    public void autonomousInit()
    {
        prepEncodersForProfile();
        setSideProfile();
    }

    public void autonomousPeriodic()
    {
        startMotionProfilesLoop();
    }

    public void testInit()
    {
        prepEncodersForProfile();
        setStraightProfile();
    }

    public void testPeriodic()
    {
        startMotionProfilesLoop();
    }

    public void teleopInit()
    {
        setMotorsDriveMode();
        resetEncoderPosition();
        resetTimer();
    }

    public void teleopPeriodic()
    {
        joystickControl();
        diagnostics();

    }

    public void disabledPeriodic()
    {
        disableMotors();
    }
}