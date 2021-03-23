def display_images(outputs, inputs=None, gt=None, is_colormap=True, is_rescale=True):
    import matplotlib.pyplot as plt
    import skimage
    from skimage.transform import resize
    import numpy as np

    plasma = plt.get_cmap('plasma')

    shape = (outputs[0].shape[0], outputs[0].shape[1], 3)
    
    all_images = []

    for i in range(outputs.shape[0]):
        imgs = []
        
        #if isinstance(inputs, (list, tuple, np.ndarray)):
        #    x = to_multichannel(inputs[i])
        #    x = resize(x, shape, preserve_range=True, mode='reflect', anti_aliasing=True )
        #    imgs.append(x)

        #if isinstance(gt, (list, tuple, np.ndarray)):
        #    x = to_multichannel(gt[i])
        #    x = resize(x, shape, preserve_range=True, mode='reflect', anti_aliasing=True )
        #    imgs.append(x)

        if is_colormap:
            rescaled = outputs[i][:,:,0]
            if is_rescale:
                rescaled = rescaled - np.min(rescaled)
                rescaled = rescaled / np.max(rescaled)
            imgs.append(plasma(rescaled)[:,:,:3])
        else:
            imgs.append(to_multichannel(outputs[i]))

        img_set = np.hstack(imgs)
        all_images.append(img_set)

    all_images = np.stack(all_images)
    return np.asarray(all_images[0])
