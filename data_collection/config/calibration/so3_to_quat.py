import mrob
import numpy as np




# =================================================================================
# Transformation of the aruco-VLP by Kishkun to algorithm. It asumes camera is in standard Z(depth)
# =================================================================================
# TODO this should be added to the axis permutation. Anyway now it does not work
print('\n Aruco- VLP to basler')
a = np.array([[0.99967648, 0.01050747, -0.02316329],
              [-0.00981060, 0.99950187, 0.02999605],
              [0.02346693, -0.02975910, 0.99928159]])
R = mrob.geometry.SO3(a)
R.print()
q = mrob.geometry.so3_to_quat(a)
print('quat compoenets =\n', q)



# =================================================================================
# Azure to VLP 
# =================================================================================
# Transformation to change the azis to point to the Z direction. From camera to the base
print('\n Cameras axis in quat')
a = np.array([[0, 0, 1],
              [-1, 0, 0],
              [0, -1, 0]])
              
q = mrob.geometry.so3_to_quat(a)
print('quat components Camera to Base =\n', q)




print('\nAzure to Velodyne')
# Transformation provided by ICP VLP-Azu provided by anastasiia icp p2pl. 
# TODO need more pairs of pcds and a new transformation
b = np.array([[-0.07230757, -0.99737818,  0.00289465,  0.47134932],
 [-0.08951381,  0.00359895, -0.99597908, -0.06401308],
 [ 0.99335738, -0.07227594, -0.08953935, -0.03230963],
 [ 0.        ,  0.         , 0.       ,   1.        ]])
 
T_icp = mrob.geometry.SE3(b).inv()


# look for Azure depth_|camera_link to camera base, this is specified in the contructor at the ros node code.
b = np.array([0,0,1.8e-3])
a = np.array([0.525482745499, -0.525482745499, 0.473146789256, -0.473146789256])
R_raw = mrob.geometry.quat_to_so3(a)

T_construction = mrob.geometry.SE3(mrob.geometry.SO3(R_raw),b)


Tx = T_icp.mul(T_construction.inv())
Tx.print()
print('quat components Azure base(camera_base) to Base_link =\n', mrob.geometry.so3_to_quat(Tx.R()))




# =================================================================================
# Azure to RGB (smartphone
# =================================================================================
# this code was provided in the repository multiview-camera-depth calibration
# It requires to:
#  1) solve the pnp problem of the coordinates in the chessboard
#  2) segment planes for Azure (or VLP)
#  3) run mrob point2plane
#
# Code is in camera-depth-calibration.ipynb
# TODO create a script that runs the above process in a single notebook.

# We tried two approaches:
# 1) the first one is obtaining plane parameters from the pointcloud and align points from PnP
# 2) Plane parameters from PnP smartphone and align them with points (decimated) from point cloud

# The solution of 1, as reported in the np is
print('\nCamera lidar solution:')
s1 = np.array([[ 0.99728918, -0.00370076, -0.07348877,  0.2270376 ],
               [-0.00479579,  0.99334178, -0.11510475, -0.02529519],
               [ 0.07342544,  0.11514515,  0.99063126, -0.04095365],
               [ 0.        ,  0.        ,  0.       ,   1.        ]])
smart_T1_azure = mrob.geometry.SE3(s1)

s2 = np.array([[ 0.99696022, -0.00162089,  0.07789536, -0.2221667 ],
               [-0.00699329,  0.99388633,  0.11018643,  0.03609816],
               [-0.07759773, -0.11039623,  0.99085381,  0.05171391],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
azure_T2_smart = mrob.geometry.SE3(s2)
print('distance rotation between both solutions:\n',azure_T2_smart.distance_rotation(smart_T1_azure.inv()))
print('distance translation between both solutions:\n', azure_T2_smart.distance_trans(smart_T1_azure.inv()))


# calcuating transformation from base to Smartphone: B^T_S = B^T_A * A^T_Ad * Ad^T_S
# Ad^T_S this is just calculated, from smartphone to Azxure depth
# A^T_Ad Azure depth to azure camera (T_construction)
# B^T_A From Azure camera to Baselink, calculated above in point cloud (Tx)
base_Tx_smart = T_icp.mul(smart_T1_azure)
print('base_Tx_smart:')
base_Tx_smart.print()
print('quat components Smartphone RGB to Base_link =\n', mrob.geometry.so3_to_quat(base_Tx_smart.R()))


