import numpy as np
import scipy.misc
def labels_to_cityscapes_palette(path):
    """
    Convert an image containing CARLA semantic segmentation labels to
    Cityscapes palette.
    """
    # transform the saved data into single array
    #use the code,the semantic segmeantation data is saved as [r,g,b] format 
    array= scipy.misc.imread(path)[:, :, 0]
    classes = {
        0: [0, 0, 0],         # None
        1: [70, 70, 70],      # Buildings
        2: [190, 153, 153],   # Fences
        3: [72, 0, 90],       # Other
        4: [220, 20, 60],     # Pedestrians
        5: [153, 153, 153],   # Poles
        6: [157, 234, 50],    # RoadLines
        7: [128, 64, 128],    # Roads
        8: [244, 35, 232],    # Sidewalks
        9: [107, 142, 35],    # Vegetation
        10: [0, 0, 255],      # Vehicles
        11: [102, 102, 156],  # Walls
        12: [220, 220, 0]     # TrafficSigns
    }
    result = np.zeros((array.shape[0], array.shape[1], 3))
    for key, value in classes.items():
        result[np.where(array == key)] = value
    return result
#RuntimeError: Could not execute image viewer.
#in windows,if you use the scipy.misc.imshow()----->error
#change the module,plt.imshow()
#however you have to change the data to BGR mode
