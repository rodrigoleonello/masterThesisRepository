# Package tum_ardrone with LQR and Hinf controller

This package is based on the following publications:

- [Scale-Aware Navigation of a Low-Cost Quadrocopter with a Monocular Camera](https://vision.in.tum.de/_media/spezial/bib/engel14ras.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Camera-Based Navigation of a Low-Cost Quadrocopter](https://vision.in.tum.de/_media/spezial/bib/engel12iros.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing](https://vision.in.tum.de/_media/spezial/bib/engel12vicomor.pdf) (J. Engel, J. Sturm, D. Cremers) 

Basically, the implementation found in the [tum_ardrone](https://github.com/tum-vision/tum_ardrone) package was used to test different controllers.

The state feedback gains was calculated using the matlab files, seen in the matlab folder 

You can find a [video](https://youtu.be/Af73_2scaxk) on *youtube*.

## Installation

### with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/rodrigoleonello/masterThesisRepository -b main
cd ..
catkin_make
```

## Quick start

### Launch the nodes

``` bash
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone.launch
```

## Results

### LQR controller

- Reference response
<img src="https://raw.githubusercontent.com/rodrigoleonello/masterThesisRepository/main/results/lqr/LQR8POSES.png" width="600">

- Control signals
<img src="https://raw.githubusercontent.com/rodrigoleonello/masterThesisRepository/main/results/lqr/LQR8CONTROLE.png" width="600">

- 3D response
<img src="https://raw.githubusercontent.com/rodrigoleonello/masterThesisRepository/main/results/lqr/3D.png" width="600">
