package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  Thread m_visionThread;

  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private CANSparkMax m_rightMotor = new CANSparkMax(3, MotorType.kBrushed);
  private DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  XboxController m_controller = new XboxController(0);

  // encoder
     private DigitalInput irSensor; // The IR sensor connected to a digital input
    private int holeCount = 0; // Counts detected holes
    private int cycles = 0; // Counts complete cycles
    private boolean lastSensorState = false; // Tracks previous sensor state
 // Configuration
 private static final double DISC_DIAMETER_CM = 15.0; // Diameter of the disc in cm
 private static final int HOLES_PER_CYCLE = 6; // Holes per one cycle




  private CANSparkMax joint_arm = new CANSparkMax(6, MotorType.kBrushed);
  private CANSparkMax slider = new CANSparkMax(5, MotorType.kBrushed);

  private AnalogInput m_analogInput = new AnalogInput(0); // Create AnalogInput for pin 0

  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    irSensor = new DigitalInput(0); // Initialize IR sensor on digital port 0

    m_rightMotor.setInverted(false);
    m_leftMotor.setInverted(true);
    // slider.setIdleMode(IdleMode.kBrake);


  
  }

  @Override
  public void teleopPeriodic() {
    double speed = m_controller.getLeftY();
    double turn = m_controller.getRightX();
    double boost = m_controller.getRightTriggerAxis();

    // Apply manual joystick control in teleop mode
    m_robotDrive.arcadeDrive((speed) * (boost + 0.5), (-turn) * (boost + 0.5));


    // joint arm logic using lt and rt


    if (m_controller.getLeftBumper()) {
      slider.set(-0.25);
    } else if (m_controller.getRightBumper()) {
      slider.set(0.25);

    } else {
      slider.set(0);
      
    }

    
   
    
    // slider logic using arrows
    // slider.set(0.5);

    if (m_controller.getPOV() == 0) {
      joint_arm.set(0.5);
    } else if (m_controller.getPOV() == 180) {
      joint_arm.set(-0.5);
    } else {
      joint_arm.set(0);
    }

    if(m_controller.getAButton()){
      // driveToDistance(100.0, 60);
      followPath( );
      // turnInPlace(180.0, 0.75, 10.0);  // Turn 180 degrees, 50% speed, for 10 seconds

    }


    boolean currentSensorState = irSensor.get(); // Get the current state of the IR sensor

    // Check for rising edge (hole detection)
    if (currentSensorState && !lastSensorState) {
        holeCount++;
        if (holeCount >= HOLES_PER_CYCLE) {
            cycles++;
            holeCount = 0; // Reset hole count after a full cycle
        }
    }

    // Update the last sensor state for edge detection
    lastSensorState = currentSensorState;

    // Calculate distance traveled
    double distanceTraveled = calculateDistanceTraveled(cycles);
    System.out.println("Distance Traveled: " + distanceTraveled + " cm");

    
    
    // Dashboard
    SmartDashboard.putNumber("Distance Traveld", distanceTraveled);
    SmartDashboard.putNumber("input 0", m_analogInput.getValue());
    SmartDashboard.putNumber("Joystick Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Joystick X", m_controller.getRightX());
    SmartDashboard.putNumber("Boost", m_controller.getRightTriggerAxis());
    SmartDashboard.putNumber("Analog Input", m_analogInput.getAverageVoltage()); // Display analog input value
    // SmartDashboard.putNumber("Center X", centerX);  // Display the X position of the blue object
    SmartDashboard.putData("Robot Drive", m_robotDrive);
  }


  private double calculateDistanceTraveled(int cycles) {
    double discCircumference = Math.PI * DISC_DIAMETER_CM;
    return cycles * discCircumference; // Distance is cycles * circumference
}

  @Override
  public void autonomousInit() {
    // Any initialization for autonomous can be added here.
  }

  @Override
  public void autonomousPeriodic() {
    // Modify turn based on the blue object position
    double turnAdjustment = 0.0;
    double speed = 0.5; // Constant speed forward

    // if (centerX > 0) {
    //   double error = (centerX - (CAMERA_WIDTH / 2)) / (CAMERA_WIDTH / 2); // Error: -1 (left) to 1 (right)
    //   turnAdjustment = -error * 0.5;  // Adjust this value to control how aggressively the robot turns
    // }

    // Drive the robot autonomously towards the blue object
    m_robotDrive.arcadeDrive(-speed, turnAdjustment);

    SmartDashboard.putNumber("input 0", m_analogInput.getValue());
    SmartDashboard.putNumber("Analog Input", m_analogInput.getAverageVoltage()); // Display analog input value
    // SmartDashboard.putNumber("Center X", centerX);  // Display the X position of the blue object
    SmartDashboard.putData("Robot Drive", m_robotDrive);
  }

  public void followPath() {
    // Step 1: Drive forward 200 cm
    driveToDistance(200.0, 0.0);  // Drive forward 200 cm, with 0-degree turn (go straight)

    // Step 2: Turn 180 degrees to face the opposite direction
    turnInPlace(180.0, 0.75, 1.5);  // Turn 180 degrees, 50% speed, for 10 seconds

    // Step 3: Drive back 200 cm to the starting point
    driveToDistance(200.0, 0.0);  // Drive forward 200 cm again (after the turn, this moves back)
}



public void turnInPlace(double directionDegrees, double turnSpeed, double timeSeconds) {
  double turnAdjustment = (directionDegrees / Math.abs(directionDegrees)) * turnSpeed;  // Set turning direction and speed

  // Turn the robot for the specified time
  long startTime = System.currentTimeMillis();  // Get the current time
  long duration = (long) (timeSeconds * 1000);  // Convert time to milliseconds

  while (System.currentTimeMillis() - startTime < duration) {
      m_robotDrive.arcadeDrive(0.0, turnAdjustment);  // No forward movement, only turning
  }

  // Stop the robot after the time has elapsed
  m_robotDrive.arcadeDrive(0.0, 0.0);
}




  public void driveToDistance(double targetDistanceCm, double directionDegrees) {
    // Reset cycle and hole counts at the start
    holeCount = 0;
    cycles = 0;

    // Calculate turn adjustment for direction
    double turnAdjustment = directionDegrees / 180.0; // Scale the turn value (-1 to 1 based on degrees)

    // Keep driving until the target distance is reached
    while (calculateDistanceTraveled(cycles) < targetDistanceCm) {
        // Drive the robot with the given direction (speed = 0.5, turn based on directionDegrees)
        m_robotDrive.arcadeDrive(0.65, turnAdjustment); // Move forward with a constant speed (0.5) and direction
        
        // Update sensor data and calculate distance traveled
        boolean currentSensorState = irSensor.get(); // Get current state of the IR sensor
        
        // Check for rising edge (hole detection)
        if (currentSensorState && !lastSensorState) {
            holeCount++;
            if (holeCount >= HOLES_PER_CYCLE) {
                cycles++;
                holeCount = 0; // Reset hole count after a full cycle
            }
        }

        // Update the last sensor state for edge detection
        lastSensorState = currentSensorState;
    }

    // Stop the robot after reaching the target distance
    m_robotDrive.arcadeDrive(0.0, 0.0);
}


}
