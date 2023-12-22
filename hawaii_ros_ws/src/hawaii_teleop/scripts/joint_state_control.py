from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
import time
import serial, threading, queue, sys

DT = 0.033

def mock_serial_data(data_queue):
    value = 0
    increment = 0.01
    increasing = True
    while True:

        if increasing:
            value += increment
        else:
            value -= increment
        if value >= 0.5:  
           increasing = False
        elif value <=0:
            increasing = True

        array = [value]*7
        data_queue.put(array)
        time.sleep(DT)

# Function to continuously read data from the serial port
def read_from_serial(serial_port, data_queue):
    # will need to do more once the payload structure is finalized
    while True:
        if serial_port.in_waiting > 0:
            data = serial_port.readline().decode().strip()
            data_queue.put(data)  # Put the data into the shared queue

def teleop(robot_side, data_queue, init_node):
    student_arm = InterbotixManipulatorXS(robot_model="Student_Arm", group_name="arm", gripper_name="gripper", robot_name=f'student_{robot_side}', init_node=init_node)
    gripper_command = JointSingleCommand(name="gripper")

    while True:
        # sync joint positions
        teacher_joint_states = data_queue.get()
        print(f"Thread {robot_side} processed:", teacher_joint_states)
        
        student_arm.arm.set_joint_positions(teacher_joint_states[:-1], blocking=False)
        # sync gripper positions
        student_gripper_target = teacher_joint_states[-1]
        gripper_command.cmd = student_gripper_target
        student_arm.gripper.core.pub_single.publish(gripper_command)
        # sleep DT
        time.sleep(DT)

if __name__=='__main__':

    # ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

    # Shared queue for passing data between threads
    shared_data_queue = queue.Queue()
    side = sys.argv[1]

    # Create threads for reading and processing in Thread 1 and Thread 2
    # read_thread = threading.Thread(target=read_from_serial, args=(ser, shared_data_queue))
    write_thread = threading.Thread(target=mock_serial_data, args=(shared_data_queue,))
    process_thread_1 = threading.Thread(target=teleop, args=(side ,shared_data_queue, True))

    # Start all threads
    # read_thread.start()
    write_thread.start()
    process_thread_1.start()

    # read_thread.join()
    write_thread.join()
    process_thread_1.join()