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
Please make sure to indicate decimals as floats.

### Configuration
There are a bunch of simple configurations that can be made to ensure that the PID controller runs smoothly.

#### Set Target Range
Since the Lego sensors are pretty inaccurate from time to time, it might help to set a range around the desired target in which no error correction is made.
This can be done in two ways:
```java
  PID.setTarget(0.5F);
  PID.setTargetRange(0.1F); //sets target to anything between 0.4 and 0.6
```
or simply like this:
```java
  PID.setTargetRange(0.4F, 0.6F); //sets target to anything between 0.4 and 0.6
```

#### Limit Output
If you are using the PID controller to directly steer the motors, you might want to set a range for the output to avoid crazy behaviour.
```java
  PID.setOutputRange(0, 160); 
  float out = PID.getOutput(sensorInput); //out will always be between 0 and 160, depending on the situation.
```

#### Set maximal I Output
If you find your robot to be massively overshooting, you can limit the output of the integral part with:
```java
  PID.setMaxIOutput(0.2F);
```

#### Use Kalman Filter
The PID controller does not use the Kalman filter by default. You can active it by giving it a value between 0 and 1.
```java
  PID.setKalmanFilter(0.5F);
```
In this case, the output of the PID controller is 50% determined by the actual sensory input and 50% determined by past sensory input.
This can help to ignore wrong sensory inputs.
