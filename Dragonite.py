import numpy as np
import cv2
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
# Initiate surf detector
#surf = cv2.xfeatures2d.SIFT_create()
surf = cv2.xfeatures2d.SURF_create(hessianThreshold=2000)
dragonite=cv2.imread("dragonite.jpg")
kp1, des1 = surf.detectAndCompute(dragonite,None)

def detect_features(img,kp1,des1,frame,frame_g):
    MIN_MATCH_COUNT = 10
   
    # find the keypoints and descriptors with SURF in each image
    
    kp2, des2 = surf.detectAndCompute(frame_g,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    # Configura o algoritmo de casamento de features
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good)>MIN_MATCH_COUNT:
        # Separa os bons matches na origem e no destino
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


        # Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img.shape[0],img.shape[1]

        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        # Transforma os pontos da imagem origem para onde estao na imagem destino
        dst = cv2.perspectiveTransform(pts,M)

        # Desenha as linhas
        frame = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA)
	
	# Returns
	# Centro da frame
	centro = (frame.shape[0]//2, frame.shape[1]//2)
	#print(centro)
	# Centro da imagem
	x=0
	y=0
	for i in range(0, len(np.int32(dst)-1)):
		x += np.int32(dst)[i][0][0]
		y += np.int32(dst)[i][0][1]
	media = (x//4, y//4)
	print("x: {}".format(x))
	print("y: {}".format(y))
	print("media: {}".format(media))
	# Area da imagem
        
    else:
        print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        matchesMask = None

    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)
    #frame=drawMatches(img,kp1,frame_g,kp2,good[:20])
    
    return frame


while(True):
    #print(timer)
    # Capture frame-by-frame
    #print("Novo frame")
    ret, frame = cap.read()

    frame_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frame_gray=cv2.medianBlur(frame_gray,5)
    #testar o bilateral filtering
  

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html
    frame=detect_features(dragonite,kp1, des1,frame,frame_gray)
    
    # Display the resulting frame
    cv2.imshow('original',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #print("No circles were found")
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
