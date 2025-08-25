# Team Viper – WRO 2025

We’re ***Team Viper***, a first-year WRO team learning and building as we go. Our name comes from the Python code that powers our robot—and the viper snake, known for its precision and adaptability. We try to reflect that in our design: thoughtful, responsive and always improving. Our robot features a modular split chassis design, custom enclosure, and symbolic touches that make it feel personal. Each of us brings different strengths, from coding and mechanical layout to documentation and expressive feedback. We’re still figuring things out, but we’re proud of what we’ve built—and excited to learn more.

---

## Power System

Our robot uses a **3 cell barrery pack** to supply power to the robot, split by a buck converter for the different voltage requirements:

- A **battery pack** powers the DC motor, **L298N** motor controller and the **MG996R servo motor** for steering.
- From the battery pack a buck converter steps  down the voltage, then supplies the Raspberry Pi, which acts as the central processing unit.
- The Raspberry Pi then powers:
  - A **time-of-flight sensor array** via an I²C multiplexer
  - A **TCS34725 color sensor** for surface detection

This setup helps us manage power distribution efficiently while keeping sensitive components isolated from motor noise and voltage spikes.

---

## Sensing System

To handle environmental sensing, our robot uses:

- **Five VL53L0X time-of-flight sensors**:
  - One facing forward
  - Two on the left (front and rear)
  - Two on the right (front and rear)

- A **TCS34725 color sensor**, mounted on the front underside of the bottom chassis, used for surface recognition and line tracking.

All sensors are managed through a **DFRobot I²C multiplexer**, which allows multiple I²C devices to communicate with the Raspberry Pi efficiently.

For obstacle management, we use a **Raspberry Pi Camera Module 3**, mounted at the front. It provides high-resolution imaging and fast frame rates, enabling real-time visual detection of obstacles.

---

## Drivetrain and Steering

- **Steering** is controlled by an **MG996R servo motor** mounted at the front.
- **Propulsion** is provided by a **12V DC motor** connected to a gearbox.
- We use an **L298N dual H-bridge motor controller** to manage the DC motor’s direction and speed.
- The **wheelbase** has been shortened from the original model we were provided, improving maneuverability and allowing for a more compact chassis layout.

---

## Symbolic and Expressive Design

We’ve added symbolic elements to our robot, including custom plaques and expressive LED feedback, to reflect our team’s identity and design philosophy. These touches help us blend technical clarity with creative expression. Something we value deeply as a team.

---

Thanks for checking out our robot! We’re excited to keep learning, iterating, and improving as we grow through our first year in Future Engineers.

PS. 
Special thanks to the following:

 - **Sudo apt install win -y** for sharing the design for the original VL53L0X mounts
 - Our Tech Educator; **MR. Danie Olivier** for printing our part mounts
 - **Tony Williams** for supplying some of the parts
 - **Johan Benadie** for supplying us with a template design and teaching us how the robot works
 - **Jarret Williams** for helping us and guiding us in the building of our project
 - **MR G.Steele** for a voice of reason and navigating the *Linux* Distribution
