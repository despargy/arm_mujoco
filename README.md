
<h1 align="center">






</h1>

<h1 align="center">
  <!-- <br>
  <a href="https://github.com/despargy/arm_mujoco"><img src="maestro_mujoco.drawio.png" alt="Maestro Mujoco" width="600"></a>
  <br> -->
  Robotic arm in  <a href="https://mujoco.org/" target="_blank">Mujoco</a>
  <br>


</h1>



<h3>Supports: UR5 </h3>
<h3 align="center"> A simple position controller for periodic motions in Mujoco.</h3> 

<p align="center">
  <a href="#Description">Description</a> •
  <a href="#Clone-and-build">Clone-Build-Execute</a> •
  <a href="#contact">Contact</a> •
  <a href="#license">License</a>
</p>


## Description

This repo can be used either as a tutorial and first interastion regarding robotic arms either..... as a scene for another package. Stay tuned!


## Clone and build(C++)

   ```sh
   git clone https://github.com/despargy/arm_mujoco.git
   ```
## Clone and Run(Python)

   ```sh
   git clone https://github.com/despargy/arm_mujoco.git
   cd arm_mujoco/src
   python3 arm.py
   ```

Build .
   ```sh
   cd arm_mujoco/src/cmake 
   mkdir build
   cd build
   cmake ..
   make
   ```


Periodic motion execution.
 ```sh
   cd arm_mujoco/src/cmake/build
   ./arm > data.csv # Run 
   cd arm_mujoco/src/
   python3 plot.py # Vizualize results
   ```

Visualize Results

```sh
   cd arm_mujoco/src/
   python3 plot.py
   ```



## Contact
   Despina-Ekaterini Argiropoulos - despinar@ics.forth.gr         

[![LinkedIn][linkedin-shield]][linkedin-url] 


[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]:https://www.linkedin.com/in/despar/


## License

MIT License