# STARDUST: Behind the Scenes

[![Watch the video](https://www.youtube.com/watch?v=CYAAUwyI8sE)](https://www.youtube.com/watch?v=CYAAUwyI8sE)

https://www.youtube.com/watch?v=CYAAUwyI8sE

## Meet the Team
Our team consists of Matthew, Luke, Arya, and Andrew. We are all freshmen, and we were so excited going into our first hackathon! None of us are CS, so this weekend was a learning curve for all of us. It was so fun to participate and be able to make a functioning product! We did not assign specific roles; however, Luke refined the CAD design of our case while Matthew, Arya, and Andrew focused on the electronics and embedded system design.

## Exigence
Initially, STARDUST originated from the idea of being able to take notes on any wall or surface. As we developed, we found many more applications for our technology. Rather than something that needs to be restricted to a specific area, STARDUST allows for the display of information in unconventional places. This has applications in unstable environments, where portable announcements or unconventional schooling is needed such as post natural disaster or displacement.

We focused on making the design as simple as possible: with the only input being a force sensor, STARDUST is inherently intuitive. It was definitely a challenge fitting everything into the small form factor, but we are glad we were able to see our vision to completion.

## Challenges & Solutions
Unsurprisingly, the STARDUST came with many challenges and learning opportunities. Thankfully, each one was packaged with incredibly interesting solutions that stuck with us after hours of troubleshooting. We’d like to share some developments that were particularly meaningful to us:

### Electrical
We really wished for the STARDUST to be wireless. However, we didn’t have reliable batteries or a voltage regulator. Ultimately we settled on a pretty funny solution. Deglove a power bank with a dremel and steal the components. To keep the power bank active, we’d constantly drain power from it using an LED (something that became very useful down the line when working with OpenCV).

In the early morning of 2 AM, we had many problems integrating wireless communications into our setup. The combination of our rapidly updating force sensor and frequent bluetooth transmissions would cause current spikes within our entire circuit. These current spikes would consistently create large voltage drops, leading to brownouts and constant loss of communications between our wireless systems. We learned to implement capacitors into our circuit to steady the voltage and prevent brownouts.

### Software
Every movement of the STARDUST pen would slightly misplace our pen’s sensitive force sensor, causing erratic and inconsistent brush strokes. To combat this, instead of setting a constant threshold force to tell the STARDUST when to draw, we created a variable threshold that would adapt to the current state of the force sensor. This variable system averages the force of the sensor when it isn’t drawing, and sets the threshold force (to create brush strokes) above that by a constant. Through this method, we were able to create a system that allowed STARDUST to accurately detect when ADDITIONAL pressure was being applied to the pen, thus prompting it to draw.

### Mechanical
We had issues with the tip of the STARDUST. Since the tip had to be fully disconnected from the rest of the body while still being restricted to the body, a simple cylinder with a semi-sphere set-up will not work, since that will rotate within the body which we cannot have since the actuator inside the tip needs to touch the force sensor. We came up with a CAD revision where a 1.5mm extrusion was added to the middle cylinder and an equal length cut was made from the outer, allowing press and push movement but restricting rotation.

We also encountered issues with packaging. Despite borrowing the compact packaging of a pre-bought power bank, getting loads of electronics and wires all into a slim form factor was still challenging. There were many times we had to resort to dremeling the interior body walls to make room for the Force Sensor Chip and ESP 32 Chip.

## What's next for STARDUST
We’d love to continue finishing up STARDUST. There were many ideas we wanted to add that we just didn’t have time for within this short period.

One such feature was the integration of multiple phones for more accurate and less cumbersome interactions. Having multiple phones connect to a website and share camera data would allow for the speaker to move freely without worry of blocking the computer vision system. Using phones from the audience would be a convenient and efficient way of improving our product’s quality.

Something else we would love to see for STARDUST is an entirely wireless setup. Currently, our projector is borrowed from one of our very generous friends. However, in the future, we hope to find a standalone, rechargeable projector to fully emphasize our position towards convenience. With this change, you wouldn’t even need a power outlet to use STARDUST.



<img width="1316" height="1177" alt="Screenshot 2025-10-19 100521" src="https://github.com/user-attachments/assets/f38b04de-4f53-4d3f-b933-6467292540e7" />
<img width="1084" height="691" alt="Screenshot 2025-10-19 100356" src="https://github.com/user-attachments/assets/265a737f-6a11-4e9d-81ec-ee9071d75e15" />
<img width="1357" height="831" alt="Screenshot 2025-10-19 100637" src="https://github.com/user-attachments/assets/b19af6db-ee65-4b84-ae0b-aa1545e93625" />
