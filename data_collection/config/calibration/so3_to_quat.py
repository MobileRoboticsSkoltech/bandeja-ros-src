import mrob
import numpy as np





# Transformation of the aruco to algorithm. It asumes camera is in standard Z(depth)
print('\n Aruco- VLP to basler')
a = np.array([[0.99967648, 0.01050747, -0.02316329],
              [-0.00981060, 0.99950187, 0.02999605],
              [0.02346693, -0.02975910, 0.99928159]])
R = mrob.geometry.SO3(a)
R.print()
q = mrob.geometry.so3_to_quat(a)
print('quat compoenets =\n', q)
# TODO this should be added to the axis permutation. Anyway now it does not work



# Transformation to change the azis to point to the Z direction. From camera to the base
print('\n Cameras axis in quat')
a = np.array([[0, 0, 1],
              [-1, 0, 0],
              [0, -1, 0]])
              
q = mrob.geometry.so3_to_quat(a)
print('quat components Camera to Base =\n', q)




print('\nAzure to Velodyne')
# Transformation provided by ICP VLP-Azu provided by anastasiia icp p2pl
b = np.array([[-0.07230757, -0.99737818,  0.00289465,  0.47134932],
 [-0.08951381,  0.00359895, -0.99597908, -0.06401308],
 [ 0.99335738, -0.07227594, -0.08953935, -0.03230963],
 [ 0.        ,  0.         , 0.       ,   1.        ]])
 
T_icp = mrob.geometry.SE3(b).inv()


# look for Azure depth_|camera_link to camera base, this is specified in the contructor
b = np.array([0,0,1.8e-3])
a = np.array([0.525482745499, -0.525482745499, 0.473146789256, -0.473146789256])
R_raw = mrob.geometry.quat_to_so3(a)

T_construction = mrob.geometry.SE3(mrob.geometry.SO3(R_raw),b)


Tx = T_icp.mul(T_construction.inv())
Tx.print()
print('quat components Azure base(camera_base) to Base_link =\n', mrob.geometry.so3_to_quat(Tx.R()))
