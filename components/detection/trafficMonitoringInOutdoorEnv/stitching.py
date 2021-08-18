
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import glob
import matplotlib


def panorama(H,img1,img2,case=1):
    if case==1:
        frame1 = np.array([[0, 0], [0, img1.shape[0]], [img2.shape[1], img2.shape[0]], [img2.shape[1], 0]],dtype=np.float32).reshape(-1, 1, 2)
        frame2 = np.array([[0, 0], [0, img1.shape[0]], [img1.shape[1], img1.shape[0]], [img1.shape[1], 0]],dtype=np.float32).reshape(-1, 1, 2)
        frame2_ = cv2.perspectiveTransform(frame2, H)
        frame = np.concatenate((frame1, frame2_), axis=0)
        [xmin, ymin] = np.array(frame.min(axis=0).ravel() - 0.5,dtype=np.int32)
        [xmax, ymax] = np.array(frame.max(axis=0).ravel() + 0.5,dtype=np.int32)
        t = [-xmin, -ymin]
        Ht = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]])
        final = cv2.warpPerspective(img1, Ht.dot(H), (xmax-xmin, ymax-ymin))
        final[t[1]:img2.shape[0]+t[1], t[0]:img2.shape[1]+t[0]] = img2
    else:
        Hinv=np.linalg.inv(H)
        dim=(img1.shape[1]+img2.shape[1],img1.shape[0])
        final = cv2.warpPerspective(img2,Hinv,dim)
        final[0:img1.shape[0],0:img1.shape[1],:] = img1
#         r = np.where(np.sum(out,(0,2)) == 0)[0]
        if(np.size(np.where(np.sum(final,(0,2)) == 0)[0])!=0):
            final = final[:,0:np.where(np.sum(final,(0,2)) == 0)[0][0],:]
    return final



def orb_match_H_esti(img1,img2,num_matches=100,case=1):
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)  
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)
    # print(matches)
    matches = sorted(matches, key = lambda x:x.distance)
    pts1  = np.array([kp1[m.queryIdx].pt for m in matches[:num_matches]],dtype=np.float32)
    pts2  = np.array([kp2[m.trainIdx].pt for m in matches[:num_matches]],dtype=np.float32)
    pts1=pts1.reshape(-1,2)
    pts2=pts2.reshape(-1,2)
    
    ##--Homography--##
    error=[]
    thresh=10000
    iterations=3000
    for h in range(iterations):
        ind=np.random.randint(low=0,high=pts1.shape[0],size=25)
        rand_pts1=pts1[ind]
        rand_pts2=pts2[ind]
        M = np.zeros( (2*rand_pts1.shape[0], 9) , dtype=np.float64)
        for i in range(rand_pts1.shape[0]):
            M[2*i] = np.array([-rand_pts1[i,0],-rand_pts1[i,1], -1, 0, 0, 0, rand_pts1[i,0]*rand_pts2[i,0], rand_pts1[i,1]*rand_pts2[i,0], rand_pts2[i,0]])
            M[2*i+1]=np.array([ 0, 0, 0, -rand_pts1[i,0], -rand_pts1[i,1], -1, rand_pts1[i,0]*rand_pts2[i,1], rand_pts1[i,1]*rand_pts2[i,1], rand_pts2[i,1]])
        U,D,Vh = np.linalg.svd(M)
        H=Vh[-1,:]
        H=H.reshape(3,3)
        H/=H[2,2]
        one=np.ones((rand_pts1.shape[0],1))
        concat=np.hstack((rand_pts1,one)).T
        reproj=np.dot(H,concat)
        reproj=(reproj/reproj[-1,:]).T
        err=np.sqrt(np.sum((reproj[:,:2]-rand_pts2)**2))
        error.append(err)
        if err<thresh:
            thresh=err
            H_best=H
    pano=panorama(H_best,img1,img2,case)
    return H_best,thresh,pano

# gsoc
def orb_match_H_esti2(img1,img2,num_matches=100,case=1):
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None) 
      
    # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # FLANN parameters
    # FLANN_INDEX_KDTREE = 1
    # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    # search_params = dict(checks=100)   # or pass empty dictionary
    # flann = cv2.FlannBasedMatcher(index_params,search_params)
    bf = cv2.BFMatcher()
    # matches = bf.match(des1,des2)
    matches = bf.knnMatch(des1,des2, k=2)
    # matches = flann.knnMatch(des1.astype(np.float32),des1.astype(np.float32),k=2)
    # print(matches)
    # matches = sorted(matches, key = lambda x:x.distance)
    good = []
    for m in matches:
      if m[0].distance < 0.7*m[1].distance:
        good.append(m)
    matches = np.asarray(good)
    # matches = np.asarray(matches)
    print(matches.shape)

    # pts1  = np.array([kp1[m.queryIdx].pt for m in matches[:num_matches]],dtype=np.float32)
    # pts2  = np.array([kp2[m.trainIdx].pt for m in matches[:num_matches]],dtype=np.float32)
    # pts1=pts1.reshape(-1,2)
    # pts2=pts2.reshape(-1,2)
    if len(matches[:,0]) >= 4:
      pts1 = np.float32([ kp1[m.queryIdx].pt for m in matches[:,0] ]).reshape(-1,1,2)
      pts2 = np.float32([ kp2[m.trainIdx].pt for m in matches[:,0] ]).reshape(-1,1,2)
    
    ##--Homography--##
    H, masked = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)
    dst = cv2.warpPerspective(img1,H,(img2.shape[1] + img1.shape[1], img2.shape[0]))
    dst[0:img2.shape[0], 0:img2.shape[1]] = img2

    # error=[]
    thresh=10000
    return H,thresh,dst

"""## RESULTS:

The above process is shown for the following test cases:
"""
####----Reading Images----####

img1=cv2.imread('./cam1_716.png')
img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2RGB)
img2=cv2.imread('./cam2_716.png')
img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2RGB)
# img1=cv2.imread('./img3_1.png')
# img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2RGB)
# img2=cv2.imread('./img3_2.png')
# img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2RGB)


print(img1.shape)
print(img2.shape)
# Method-1:
# H,error,pano=orb_match_H_esti(img2,img1,100,2)
# Method-2:
H,error,pano=orb_match_H_esti2(img2,img1,100,2)
print("H:")
print(H)
print("error:",error)
fig = plt.figure(figsize=(12.8,9.6))
plt.imshow(pano)

plt.imsave("out_stiched_working.jpg", pano)
