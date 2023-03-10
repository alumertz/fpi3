import numpy as np
import cv2 as cv


op = np.zeros(13)
width =0
height =0
menu = """
    q or Q = quit
    b or B = gaussian Blur
    e or E = canny Edges
    s or S = gradient Sobel
    i or I = brIghtness
    c or C = Contrast
    n or N = Negative
    g or G = greyscale
    d or D = reDimension to 1/4
    r or R = Rotate 90 degrees
    h or H = Horizontal mirroring
    v or V = Vertical mirroring
    w or W = start/stop video Writing (saving)
"""




def getNewFrame(op,frame):
    contrast = 1+cv.getTrackbarPos("Contrast", "Processed")/100
    brightness = cv.getTrackbarPos("Brightness", "Processed")
        
    if op[1] ==1:
        size = cv.getTrackbarPos("Gaussian Blur", "Processed")
        if size%2==0: #precisamos garantir que o tamanho do kernel seja ímpar
            size-=1
        if size<0:
            size=1
        frame = cv.GaussianBlur(frame,(size,size),0)
        
    if op[2] ==1:
    #     canny
        frame = cv.Canny(frame,50,150)
        frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
    if op[3] ==1:
    #     sobel
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        x = cv.Sobel(frame, cv.CV_8U, 1, 0, ksize=3, scale=1, delta=0, borderType=cv.BORDER_DEFAULT)
        y = cv.Sobel(frame, cv.CV_8U, 0, 1, ksize=3, scale=1, delta=0, borderType=cv.BORDER_DEFAULT)
        absx= cv.convertScaleAbs(x)
        absy = cv.convertScaleAbs(y)
        frame = cv.addWeighted(absx, 0.5, absy, 0.5,0)
        frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)

    if op[4] ==1:
        #change brightness
        frame = cv.convertScaleAbs(frame, alpha=1, beta=brightness)
        
    if op[5] ==1:
        #change contrast
        frame = cv.convertScaleAbs(frame, alpha=contrast, beta=0)   
    if op[6] ==1:
        #negative
        frame = cv.convertScaleAbs(frame, alpha=-1, beta=255)

    if op[7] ==1:
    #   grey
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
        
    if op[8] ==1:
    #     redimension
        frame = cv.resize(frame, (int(width/2),int(height/2)))

    if op[9] !=0:
    #     rotate
        frame = cv.rotate(frame,cv.ROTATE_90_CLOCKWISE)
    if op[10] ==1:
    #     mirror vertical
        frame = cv.flip(frame,0)
    if op[11] ==1:
    #     mirror horizontal
        frame = cv.flip(frame,1)
    return frame

def nothing(x):
    pass

if __name__ == '__main__':
    import sys
    import getopt

    print(menu)

    cap = cv.VideoCapture(0)
    frame = cv.imread("../data/lena.jpg")
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    else:
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        cv.namedWindow("Processed")
        cv.createTrackbar('Contrast',"Processed",0,100, nothing)
        cv.setTrackbarMin('Contrast',"Processed", -100)
        cv.setTrackbarMin('Gaussian Blur',"Processed", -100)
        cv.createTrackbar('Brightness',"Processed",0,100, nothing)
        cv.createTrackbar('Gaussian Blur',"Processed",1,100, nothing)
        cv.setTrackbarMin('Gaussian Blur',"Processed", 1)
        record = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*'MJPG'), 20, (int(width), int(height)))
        size = (int(width), int(height))

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
             
         

        pkey = cv.waitKey(1)
        
        

        if pkey == ord('Q') or pkey == ord('q'):
            break
        elif pkey == ord('B') or pkey == ord('b'):
        #   Gaussian
            op[1]= not op[1]
        elif pkey == ord('E') or pkey == ord('e'):
        #     canny
            op[2]= not op[2]
        elif pkey == ord('S') or pkey == ord('s'):
        #     sobel
            op[3]= not op[3]
        elif pkey == ord('I') or pkey == ord('i'):
        #     brightness
            op[4]= not op[4]
        elif pkey == ord('C') or pkey == ord('c'):
        #     contrast
            op[5]= not op[5]
        elif pkey == ord('N') or pkey == ord('n'):
        #     negative
            op[6]= not op[6]
        elif pkey == ord('G') or pkey == ord('g'):
        #    grey
            op[7]= not op[7]
        elif pkey == ord('D') or pkey == ord('d'):
        #     redimension
            op[8]= not op[8]
        elif pkey == ord('R') or pkey == ord('r'):
        #     rotate
            op[9]= not op[9]
        elif pkey == ord('H') or pkey == ord('h'):
        #     mirror horizontal 
            op[10]= not op[10]
        elif pkey == ord('V') or pkey == ord('v'):
        #     mirror vertical
            op[11]= not op[11]
        elif pkey == ord('W') or pkey == ord('w'):
            op[12] = not op[12]
                
                

        nframe= getNewFrame(op,frame)

        cv.imshow('Original', frame)
        cv.imshow('Processed', nframe)

        if op[12]: #if op save video is on, saves processed frame
            record.write(nframe)


    # When everything done, release the capture and writer
    record.release()
    cap.release()
    cv.destroyAllWindows()
