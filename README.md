# 🤖 6-DOF Robotic Arm Control with Flex Sensors

This repository contains the Arduino code for controlling a **6 Degrees of Freedom (DOF) robotic arm** using **flex sensors** as input. Each flex sensor corresponds to a joint on the robotic arm, allowing for intuitive human-like control.

---

## 🌟 Features

* **6-DOF Control:** Independently control the base, shoulder, elbow, wrist pitch, wrist roll, and gripper.
* **Flex Sensor Input:** Utilize analog flex sensors to map human hand/finger movements to robotic arm joints.
* **Automatic Calibration:** Includes a `calibrateFlexSensors()` function to automatically determine the `flexStraight` and `flexBent` values for accurate mapping.
* **Smooth Movement:** Implements a smoothing algorithm (`smoothingFactor`) to prevent jerky movements of the servos.
* **Adjustable Servo Limits:** Easily define the minimum and maximum angles for each servo to prevent mechanical damage.
* **Serial Debugging:** Provides detailed real-time feedback on flex sensor readings, target angles, and actual servo positions via the Serial Monitor.
* **Serial Command Interface:** Basic serial commands for "STOP", "CALIBRATE", and "RESET" for quick control and troubleshooting.
* **Emergency Stop:** A dedicated `emergencyStop()` function for immediate halting of arm movement.

---

## 🛠️ Hardware Requirements

* **Arduino Board:** (e.g., Arduino Uno, Mega, Nano)
* **6 x Servo Motors:** Standard hobby servos are generally sufficient for small to medium arms. Ensure they can handle the torque required by your arm's design.
* **6 x Flex Sensors:** (e.g., SparkFun Flex Sensor 2.2")
* **6 x 10k Ohm Resistors:** For the voltage divider circuit with the flex sensors.
* **6-DOF Robotic Arm Mechanical Structure:** Your physical arm assembly.
* **Breadboard and Jumper Wires:** For connections.
* **Power Supply:** An external power supply for the servos is highly recommended, as the Arduino's 5V pin may not provide enough current for all 6 servos simultaneously.

---

## 🔌 Wiring Diagram

Below is a general wiring guide. Please refer to your specific servo and flex sensor datasheets for exact pinouts.

### Flex Sensors

Each flex sensor requires a **voltage divider circuit**.

* Connect one end of the **flex sensor** to **5V**.
* Connect the other end of the **flex sensor** to one end of a **10k Ohm resistor**.
* Connect the other end of the **10k Ohm resistor** to **GND**.
* The **analog input pin** on the Arduino should be connected to the **junction between the flex sensor and the 10k Ohm resistor**.

| Flex Sensor   | Arduino Analog Pin |
| :------------ | :----------------- |
| Flex Sensor 1 | A0                 |
| Flex Sensor 2 | A1                 |
| Flex Sensor 3 | A2                 |
| Flex Sensor 4 | A3                 |
| Flex Sensor 5 | A4                 |
| Flex Sensor 6 | A5                 |

### Servo Motors

Connect the signal pin of each servo to its corresponding digital PWM pin on the Arduino. The VCC and GND of the servos should be connected to an **external power supply**. Make sure the GND of the external power supply is also connected to the Arduino's GND.

| Servo Joint     | Arduino Digital Pin |
| :-------------- | :------------------ |
| Base Rotation   | 3                   |
| Shoulder Joint  | 5                   |
| Elbow Joint     | 6                   |
| Wrist Pitch     | 9                   |
| Wrist Roll      | 10                  |
| Gripper         | 11                  |

---

## ⚙️ Software Setup

1.  **Install Arduino IDE:** If you don't have it, download and install the [Arduino IDE](https://www.arduino.cc/en/software).
2.  **Install Servo Library:** The `Servo.h` library is typically pre-installed with the Arduino IDE. If not, go to `Sketch > Include Library > Manage Libraries...` and search for "Servo" to install it.
3.  **Upload Code:**
    * Open the `.ino` file in the Arduino IDE.
    * Select your Arduino board from `Tools > Board`.
    * Select the correct COM port from `Tools > Port`.
    * Click the "Upload" button.

---

## 🚀 Getting Started and Calibration

After uploading the code, open the **Serial Monitor** (Baud rate: 9600).

1.  The arm will initialize to center positions.
2.  The code will prompt you to **calibrate the flex sensors**:
    * "Keep all sensors straight for 3 seconds": Ensure all flex sensors are unbent and flat. The Arduino will take readings.
    * "Now bend all sensors fully for 3 seconds": Fully bend each flex sensor. The Arduino will take readings.
3.  Once calibration is complete, the `flexStraight` and `flexBent` values will be printed to the Serial Monitor. These values are crucial for mapping the flex sensor resistance to servo angles.
4.  The robotic arm should now respond to the bending of your flex sensors.

---

## 🔧 Customization and Tuning

### Calibration Values

The `flexStraight` and `flexBent` arrays in the code are initially set to generic values. The `calibrateFlexSensors()` function will update these. However, if you notice erratic behavior or need fine-tuning, you can manually adjust these values after running a calibration and observing the output in the Serial Monitor.

    int flexStraight[6] = {200, 200, 200, 200, 200, 200}; // Initial placeholder values
    int flexBent[6] = {800, 800, 800, 800, 800, 800};     // Initial placeholder values

Servo Angle Limits

It's crucial to set appropriate servoMin and servoMax values for each joint to prevent your robotic arm from hitting its mechanical limits or damaging the servos.

    int servoMin[6] = {0, 20, 0, 0, 0, 0};      // Minimum angles for Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll, Gripper
    int servoMax[6] = {180, 160, 180, 180, 180, 180}; // Maximum angles

Adjust these based on your arm's physical design and the range of motion of your specific servos.

##Smoothing Factor

The smoothingFactor variable controls how smoothly the servos move in response to flex sensor changes.

    const int smoothingFactor = 5; // Higher = more smoothing, slower response

Experiment with this value to find a balance between responsiveness and smooth motion. A higher value will make the arm move more gradually, while a lower value will make it more responsive but potentially choppier.

##Update Interval

The updateInterval determines how often the servo positions are updated.

    const int updateInterval = 20; // Update every 20ms for smooth movement

A smaller interval means more frequent updates and potentially smoother motion, but it also increases CPU usage. For most applications, 20ms is a good starting point.

💬 ##Serial Commands

You can send the following commands via the Serial Monitor (ensure "Newline" is selected in the Serial Monitor settings):

    STOP: Triggers the emergencyStop() function, halting current servo movements.

    CALIBRATE: Reruns the calibrateFlexSensors() sequence.

    RESET: Resets all servos to their initial center position (90 degrees).

⚠️ ##Safety Notes

    Always use an external power supply for your servos. Powering them directly from the Arduino can damage your board.

    Set appropriate servo limits to prevent mechanical damage to your arm and servos.

    Be mindful of the arm's movement, especially during initial testing.

    Implement an emergency stop mechanism accessible during operation.

🤝 ##Contributing

Feel free to fork this repository, make improvements, and submit pull requests. If you encounter any issues or have suggestions, please open an issue.
