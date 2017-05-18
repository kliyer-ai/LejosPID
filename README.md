# LejosPID
This is a simple PID controller especially designed for the LEJOS environment.
It was inspired by the MiniPID controller of [Tekdemo](https://github.com/tekdemo/MiniPID-Java).

## Features
* simple PID control
* easy implementation
* integrated Kalman filter
* target range
* uses floats

## Usage
Setting up a simple PID controller could look like this:
```java
  LejosPID PID = new LejosPID(1,1,1);
  PID.setTarget(0.5F);
  
  while(true){
  float out = PID.getOutput(sensorInput);
  ...
  }
```
The order of the parameters is P, I, D. In this case, each parameter is set to 1.
Please make sure to indicate decimals as floats.

### Configuration
There are a bunch of simple configurations that can be made to ensure that the PID controller runs smoothly.

#### Scale Output
The output of the PID controller is going to be way too small to use it to control the motors. This is due to the fact that the values the LEGO sensors return are between 0 and 1 and these values are used as an input for the controller. Therefore, it is possible to scale the output to a point where it actually influences the motors significantly.
```java
  PID.setScale(1000);
```
1000 seems like a good value to start with, since the sensor outputs are between 0 and 1, and the PID controller will probably return values between 100 and 1000.

#### Limit Output
If you are using the PID controller to directly steer the motors, you might want to set a range for the **scaled** output to avoid crazy behaviour.
```java
  PID.setScale(1000);
  PID.setOutputRange(0, 160); 
  float out = PID.getOutput(sensorInput); //out will always be between 0 and 160, depending on the situation.
```

#### Set Target Range
Since the Lego sensors are pretty inaccurate from time to time, it might help to set a range around the desired target in which no error correction is made.
This can be done in two ways:
```java
  PID.setTarget(0.5F);
  PID.setTargetRange(0.1F); //sets target to anything between 0.4 and 0.6; error will be 0 in this range
```
or simply like this:
```java
  PID.setTargetRange(0.4F, 0.6F); //sets target to anything between 0.4 and 0.6
```
*I personally have no idea if this actually makes any sense...*

#### Set maximal I Output
If you find your robot to be massively overshooting, you can limit the output of the **scaled** integral part. For instance, it might be useful to first set the output range and then limit how much of the scaled output is accounted for by the integral part.
```java
  PID.setScale(1000); //exact value has to be found by trial and error
  PID.setOutputRange(0, 160); //set max output range
  PID.setMaxIOutput(40); //the integral part(summed error) can only influence the output by 40
```

#### Use Kalman Filter
The PID controller does not use the Kalman filter by default. You can active it by giving it a value between 0 and 1.
```java
  PID.setKalmanFilter(0.3F);
```
In this case, the output of the PID controller is 30% determined by the actual sensory input and 70% determined by past sensory input.
This can help to ignore wrong sensory inputs.

#### Reset
The PID control can be reset, e.g. everytime the behaviour takes action: 
```java
  PID.reset();
```
