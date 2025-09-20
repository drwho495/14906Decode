# Maintaining instructions

 * logic for animation of subsystems should be done in RobotManager, not in the teleop or in the subsystem files
 * hardware initialisation is done in the RobotManager, which passes the motors and servos into the subsystem objects.
 * each subsystem class should have public methods for controlling speeds of motors in a semi-auto mode (this does not mean giving the other files control of each individual motor, just powers for groups of motors, and positions or even states for servos.)
 * each subsystem class will control the PIDF loops (position and velocity) for motors
 * sometimes autos need to work differently in teleop, and many changes would need to be made to the robot manager to make this work. the options class will control what changes will be made for specific opmodes.