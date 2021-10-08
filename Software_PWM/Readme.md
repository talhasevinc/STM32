What is PWM?

Pulse-width modulation (PWM), or pulse-duration modulation (PDM), is a method of reducing the average power delivered by an electrical signal, by effectively chopping it up into discrete parts. The average value of voltage (and current) fed to the load is controlled by turning the switch between supply and load on and off at a fast rate. The longer the switch is on compared to the off periods, the higher the total power supplied to the load. Along with maximum power point tracking (MPPT), it is one of the primary methods of reducing the output of solar panels to that which can be utilized by a battery.PWM is particularly suited for running inertial loads such as motors, which are not as easily affected by this discrete switching, because their inertia causes them to react slowly. The PWM switching frequency has to be high enough not to affect the load, which is to say that the resultant waveform perceived by the load must be as smooth as possible.
(Quotation: wikipedia.org)

Example:

 set_PWM_Frequency(3000);        //3k Frequency   
 generate_PWM_Signal(37,20);     //Duty Cycle:37 , Signal Generate Time:20 ms
 
 
 
![3kPwm_550x350](https://user-images.githubusercontent.com/34924065/136581342-bf174950-5b53-4305-abfd-629da0d717d4.jpg)

 set_PWM_Frequency(3000);        //3k Frequency   
 generate_PWM_Signal(92,20);     //Duty Cycle:92 , Signal Generate Time:20 ms
 
 ![3kPwm2_550x350](https://user-images.githubusercontent.com/34924065/136581407-7dad2e63-de83-4946-bb6e-14c60b866f44.jpg)

 set_PWM_Frequency(4000);        //3k Frequency   
 generate_PWM_Signal(56,20);     //Duty Cycle:6 , Signal Generate Time:20 ms
 
 ![4kPwm2_550x350](https://user-images.githubusercontent.com/34924065/136581495-10591044-b023-4373-8b41-b5119d5b8e95.jpg)
