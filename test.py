#!/usr/bin/env python3

import time
from rshp import REVComm, REVConstants

def main():
    """Test program to move both motor 0 and motor 1 by 2000 encoder steps"""
    
    # Initialize communication
    comm = REVComm.REVcomm()
    comm.openActivePort()
    
    # Discover connected modules
    modules = comm.discovery()
    
    if not modules:
        print("No REV modules found!")
        return
    
    print(f"Found {len(modules)} REV module(s)")
    
    # Get first module
    module = modules[0]
    
    # Get motors 0 and 1
    motor0 = module.motors[0]
    motor1 = module.motors[1]
    
    # Initialize motors
    motor0.init()
    motor1.init()
    
    # Set motors to position control mode
    motor0.setMode(REVConstants.MODE_POSITION_TARGET, REVConstants.BRAKE_AT_ZERO)
    motor1.setMode(REVConstants.MODE_POSITION_TARGET, REVConstants.BRAKE_AT_ZERO)
    
    # Set PID coefficients for position control
    motor0.setPositionPID(1.0, 0.1, 0.05)
    motor1.setPositionPID(1.0, 0.1, 0.05)
    
    # Reset encoders to start from zero
    motor0.resetEncoder()
    motor1.resetEncoder()
    
    print("Starting motor movement test...")
    print("Moving both motors 2000 encoder steps...")
    
    # Set target position to 2000 steps with 20 step tolerance
    motor0.setTargetPosition(2000, 20)
    motor1.setTargetPosition(2000, 20)
    
    # Monitor progress
    while not (motor0.isAtTarget() and motor1.isAtTarget()):
        pos0 = motor0.getPosition()
        pos1 = motor1.getPosition()
        vel0 = motor0.getVelocity()
        vel1 = motor1.getVelocity()
        
        print(f"Motor 0: pos={pos0:4d}, vel={vel0:4d} | Motor 1: pos={pos1:4d}, vel={vel1:4d}")
        time.sleep(0.1)
    
    print("Both motors reached target position!")
    
    # Final positions
    final_pos0 = motor0.getPosition()
    final_pos1 = motor1.getPosition()
    
    print(f"Final positions - Motor 0: {final_pos0}, Motor 1: {final_pos1}")
    
    # Stop motors
    motor0.setPower(0)
    motor1.setPower(0)
    
    print("Test completed successfully!")

if __name__ == "__main__":
    main()