# DC-Motor-Control-using-L298N-Motor-Driver-with-STM32F407-Development-Kit

This Project implements DC Motor Control using the L298N Motor Driver, which has a dual H-Bridge design that allows for controlling 2 motors at once. The connections made for this
project is as follows :

![image](https://user-images.githubusercontent.com/56625259/150642238-806d46ff-1482-44ed-8eef-44eb39a52429.png)

| L298N Motor Driver | STM32F407 |
|----|----|
| IN1 | PA5 |
| IN2 | PA6 |
| GND | GND |

The connections between the Battery(9V) and the driver module:

| Battery 9V | L298N Motor Driver |
|----|----|
| Positive Terminal | Power |
| Negative Terminal | GND |

The terminals of the motor is connected to the *Output A* of the L298N Motor Driver. 
