# Landmark Detection & Robot Tracking (SLAM) 

This project is an implementation of ```SLAM``` (Simultaneous Localization and Mapping) for a 2 dimensional world. We can create a map of the environment of the robot through the combination of ```motion``` and ```sensor``` data that was gathered over time.

<br />


![Robot Location and Probability Distribution](https://video.udacity-data.com/topher/2018/May/5b073c5a_prob-dists/prob-dists.png)



<br />

## Description 

**Simultaneous Localization and Mapping** (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

Statistical techniques used to approximate the above equations include ```Kalman filters``` and particle filters (aka. Monte Carlo methods). They provide an estimation of the posterior probability function for the pose of the robot and for the parameters of the map. 

![Sense and Motion Cycle](https://i.ibb.co/5W4tW3P/sense-move.png) 

<br />


## Screenshots

Below you can see and example of 100x100 world with ```robot location``` (red o) and the place of 5 ```landmarks``` (purble x's) after many sense and move series and the ```last pose``` of the robot.

![Test Images](https://i.ibb.co/vwRjwJs/Screen-Shot-2020-05-14-at-6-07-18-PM.png)



### Prerequisites

* Previous familarity with probability and probability distribution .
* Previous familarity with Linear Algebra.


<br />

## Robot Class Methods 

###### There're two important methods in ```Robot``` class which are ```sense``` and ```move``` methods.



### Move Method 

This method is used to move the robot around the world given ```dx``` & ```dy``` (The Displacement in x and y Directions) and make sure that the robot will not move outside the world.

```python
def move(self, dx, dy):

        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise

        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            self.x = x
            self.y = y
            return True
```
<br/>


### Sense Method 

This method is used to sense the distance between each robot location ```Xi``` at a certain time step```i``` and the locations of the ```landmarks``` with a certain noise ```self.measurement_noise``` and returns a ```measurments``` list which contains the distances between current robot location and all the landmarks in the form of  ```[idx,dx,dy]``` where idx is the n'th landmark.


```python
    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 if you want all
    #        landmarks to be visible at all times
    #
    
    def sense(self):
        ''' This function does not take in any parameters, instead it references internal variables
            (such as self.landamrks) to measure the distance between the robot and any landmarks
            that the robot can see (that are within its measurement range).
            This function returns a list of landmark indices, and the measured distances (dx, dy)
            between the robot's position and said landmarks.
            This function should account for measurement_noise and measurement_range.
            One item in the returned list should be in the form: [landmark_index, dx, dy].
            '''
           
        measurements = []
 
        ## 1. compute dx and dy, the distances between the robot and the landmark
        ## 2. account for measurement noise by *adding* a noise component to dx and dy
        ##    - The noise component should be a random value between [-1.0, 1.0)*measurement_noise
        ##    - Feel free to use the function self.rand() to help calculate this noise component
        ##    - It may help to reference the `move` function for noise calculation
        ## 3. If either of the distances, dx or dy, fall outside of the internal var, measurement_range
        ##    then we cannot record them; if they do fall in the range, then add them to the measurements list
        ##    as list.append([index, dx, dy]), this format is important for data creation done later       
        
        for idx in range(self.num_landmarks):
            dx = (self.landmarks[idx][0] - self.x)
            dy = (self.landmarks[idx][1] - self.y)
            
            dx , dy = dx + (self.rand() * self.measurement_noise), dy + (self.rand() * self.measurement_noise)
            
            if (abs(dx) < self.measurement_range) and (abs(dy) < self.measurement_range):
                measurements.append([idx, dx, dy])

        return measurements
```
<br />

> code for robot class can be found in ```robot_class.py``` file.

<br />

## Authors

* **Ahmed Abd-Elbakey Ghonem** - [**Github**](https://github.com/3ba2ii)


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details




