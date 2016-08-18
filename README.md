# SWIM
Underwater SWarm sIMulator (SWIM)

![Alt text](/images/Complex.png?raw=true "Snapshot of Complex-Environment mode")

## Credits
Special thanks to Saleh Ibrahim @saleh860; the original author of the `core/network` library.

## About

SWIM is a Java 3D simulator for underwater swarms of Autonomous Underwater Vehicles (AUVs). It is based on jMonkeyEngine game development engine which utilizes jBullet (a physics engine) internally to provide physics support. 

It follows a balanced design approach between simulators that model swarms a point masses that can move omni-directionally and simulators that focus on the internal workings of individual robots and can simulate highly sophisticated operations running internally in the robot. In SWIM, physical vehicle constrains are taken into consideration, thus AUVs are not modeled as point masses but instead as robots with limited degrees of freedom. At the same time a considerable level of intelligence is built into each individual agent to enable autonomous decision making. The simulator is also equipped with numerous debugging capabilities that ease the development to a great extent and also can serve as means for generating visually appealing descriptors of different parameters.

SWIM supports five simulation modes that can be run independently: SelfOrganization-Mode, Search-Mode, TaskAllocation-Mode, Integration-Mode, and GeneralTests-Mode. The first three are used whenever their respective algorithms need to be tested in isolation from the full mission. These modes provide the proper context to apply the associated mission stage. For example, in TaskAllocation-Mode, the vehicles are placed directly near to the target to allow task allocation algorithms to start execution the moment the simulation is run. Integration-Mode uses the selected integration algorithm to connect different mission stages and allow for a complete mission test. GeneralTests-Mode was created to allow developers/researchers to test incomplete algorithms or do any tests that do not necessarily lie under any of the above categories.

Additionally, SWIM has two environment-complexity modes: Complex-Environment, and Simple-Platform modes. The first builds an environment that is more realistic: it uses a peaky terrain for sea-floor, and realistically looking water with waves that can have adjustable hight and direction and gives the underwater effect when the camera is moved below the sea surface. It also has waves sound effect that makes the scene more natural. This mode is however not always the best option, especially when large swarm sizes are to be used or the algorithms being tested are computationally intensive. The second mode abstracts many of these effects away and only uses a simple flat platform for tests.

In addition to the default tests that can be run in each simulation mode, there is a TestRunner class in the swim.test package that can be used for bulk automated tests. This class runs in the main thread and repeatedly creates instances of another thread (SimRunner) that runs the selected simulation. After creating every instance, it waits until its done doing it job and then starts a new thread. This enables running the same simulation multiple times for validation and result averaging purposes. after each run a set of text files containing the results for different measure are generated and numbered by the run's number. Further processing of these files can then be done by the researcher.

![Alt text](/images/Complex_2.png?raw=true "Another snapshot showing the underwater view")

## Authors

* **Sherif Tolba** - [website](https://sheriftolba.com/)
* **Saleh Ibrahim** - [GitHub](https://github.com/Saleh860)

See also the list of [contributors](https://github.com/aghilmort/swim/contributors) who participated in this project.

## License

This project is licensed under the MIT License
