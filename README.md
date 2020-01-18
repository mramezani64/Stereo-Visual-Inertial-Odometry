# Stereo-Visual-Inertial-Odometry

This code tightly couples the visual information coming from a stereo camera and imu measurements via Multi-State Constraint Kalman Filter (MSCKF). 

## How to use the code
To work with this code:
```
- Open S_MSCKF.m file and change the directories based upon where the code is stored.
- Many KITTI datasets have been tested for this code. Uncomment one of the datasets you would like to run the software for.
```
If wanted to use the other KITTI datasets, you should download the data from KITTI datasets http://www.cvlibs.net/datasets/kitti/raw_data.php and use contents of kitti_extraction to track features and have them stored in a specific .mat file. 

Note: This code was originally developed by Lee E Clement for mono-msckf (Clement, Lee E., et al. "The battle for filter supremacy: A comparative study of the multi-state constraint kalman filter and the sliding window filter." 2015 12th Conference on Computer and Robot Vision. IEEE, 2015.). 

## Credits
Please cite properly if this code used for any academic and non-academic purposes.

```@article{ramezani2018vehicle,
   title={Vehicle positioning in GNSS-deprived urban areas by stereo visual-inertial odometry},
   author={Ramezani, Milad and Khoshelham, Kourosh},
   journal={IEEE Transactions on Intelligent Vehicles},
   volume={3},
   number={2},
   pages={208--217},
   year={2018},
   publisher={IEEE}
 }
