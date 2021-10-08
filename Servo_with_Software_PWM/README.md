The working principle of servo motors is provided by PWM. Servo motors perform angular movements with specific PWM signal and Duty cycle. 
It is adjusted according to the signal seen in the image below in the library used. Initially, TIM2 was activated in the IOC file and the 
necessary frequency generation was performed with this timer.

Period:20 ms
Duty Cycle: %2.5-12.5 (-90 +90)

![servo](https://user-images.githubusercontent.com/34924065/136559164-034028ae-f794-4a45-aff9-919a8e12c42a.jpeg)

EXAMPLE

When run 'servoMove(0);' command;

![Servo25](https://user-images.githubusercontent.com/34924065/136580450-10b3524d-8b06-4b59-bd6f-99f02a5cff4f.jpg)

When run 'servoMove(90);' command;

![Servo75](https://user-images.githubusercontent.com/34924065/136580494-edee96d4-710c-4ee4-892a-0f28ad761afa.jpg)

When run 'servoMove(180);' command;

![Servo125](https://user-images.githubusercontent.com/34924065/136580521-17b0ed08-eb0f-46ea-a781-fd32f854aa6d.jpg)
