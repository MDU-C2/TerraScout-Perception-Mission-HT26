#Will receive a perfect simulated OAK-D pointcloud
#And output a noisy version

import numpy as np

# Parameters are taken from: https://github.com/luxonis/luxonis-isaac-sim/blob/main/README.md
noiseMean = 0.25
noiseSigma = 0.2

focalLengthPixel = 805
baseline = 0.075 #(in meters)

minDepth = 0.6
maxDepth = 12

maxDisparityPixel = 95

#My own
#randomRemoval = 0.02
 
def simRealDepth(perfect_cloud):

    noisy = perfect_cloud.copy()

    #Will add both random and systematic depth error
    # Gazebo's sensor is attached to oakd_link (base FLU); forward depth is +X.
    disparities = focalLengthPixel * baseline / noisy[:, :, 0]
    disparities += np.random.normal(noiseMean, noiseSigma, size=disparities.shape)
    noisy[:, :, 0] = focalLengthPixel * baseline / disparities

    # Remove points outside the simulated camera range.
    depth = noisy[:, :, 0]

    invalid = (depth < minDepth) | (depth > maxDepth)
    noisy[invalid] = np.nan

    #TODO
    #Remove from sharper edges?

    #TODO
    #Randomly remove?

    return noisy