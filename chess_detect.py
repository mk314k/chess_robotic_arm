import numpy as np
import cv2
from Ser_com import send_data

b_rect=(0,0,1000,1000)
#cap=cv2.VideoCapture("/Users/kartikeshmishra/Projects/ttt/chess_cv/chess3.mov")
cap=cv2.VideoCapture(0)

gray_thres_min=np.array([0],np.uint8)
gray_thres_mx=np.array([90],np.uint8)

chess=[[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None]]

chess_int=[[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None],
[None,None,None,None,None,None,None,None]]


# white_piece=np.zeros((100,100,3),np.uint8)
# for i in range(100):
#     for j in range(100):
#         white_piece[i][j]=(255,0,0)
# black_piece=cv2.cvtColor(white_piece,cv2.COLOR_BGR2RGB)



if (cap.isOpened()== False): 
  print("Error Reading Video")

def detect_piece(image,i,j):
    image_gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    img_blur=cv2.GaussianBlur(image_gray,(11,11),cv2.BORDER_DEFAULT)
    img_thresh=cv2.adaptiveThreshold(img_blur,40,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    #img_thresh=cv2.inRange(img_blur,gray_thres_min,gray_thres_mx)
    img_canny=cv2.dilate(cv2.Canny(img_thresh,100,225),None)
    c,h=cv2.findContours(img_canny,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    area_max=0
    for i in range(len(c)):
        area=cv2.contourArea(c[i])
        if area_max<area:
            area_max=area
    if area_max>=2000:
        try:
#             data=str(i)
#             if i>0:
#                 data="+"+data
#             if j>0:
#                 data=data+"+"
#             data=data+str(j)
            data=16*i+j
            send_data(data,10000)
        except:
            print("data not sent")
        finally:
            cv2.imshow("found",img_thresh)
            return 1

   
while(cap.isOpened()):
    ret,frame=cap.read()
    if ret==True:
        frame_gray=cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
        frame_gray_blur=cv2.medianBlur(frame_gray,19,cv2.BORDER_DEFAULT)

        image_area=frame.shape[0]*frame.shape[1]

        #gray_border_thresh=cv2.inRange(frame_gray_blur,gray_thres_min,gray_thres_mx)
        gray_border_thresh=cv2.adaptiveThreshold(frame_gray_blur,90,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        cv2.imshow("chess gray border",gray_border_thresh)

        frame_edges=cv2.Canny(gray_border_thresh,100,225)
        cv2.imshow("edges_frame",frame_edges)

        c,h=cv2.findContours(cv2.dilate(frame_edges,None),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)

        area_max=0
        largest_c_index=None
        for i in range(len(c)):
            area=cv2.contourArea(c[i])
            if area_max<area:
                area_max=area
                largest_c_index=i
        
        if largest_c_index and area_max>0.25*image_area:
            b_rect_temp=b_rect
            b_rect=cv2.boundingRect(c[largest_c_index])
            if b_rect[0]<b_rect_temp[0] or b_rect[1]<b_rect_temp[1] or b_rect[2]>b_rect_temp[2] or b_rect[3]>b_rect_temp[3]:
                b_rect=b_rect_temp
            # cv2.drawContours(frame,c,largest_c_index,(255,0,0))
            # cv2.drawContours(frame_gray,c,largest_c_index,(255,0,0))
            #print(b_rect,"area",area_max)

        if b_rect[2]>b_rect[0] and b_rect[3]>b_rect[1]:
            chess_board=frame[b_rect[1]:b_rect[1]+b_rect[3],b_rect[0]:b_rect[0]+b_rect[2]]
            chess_board=cv2.resize(chess_board,(840,840))
            chess_bord_2=chess_board[20:820,20:820]
            cv2.imshow("chess_board",chess_board)
            for i in range(8):
                for j in range(8):
                    chess[i][j]=chess_bord_2[100*i:100*(i+1),100*j:100*(j+1)]
                    chess_int[i][j]=detect_piece(chess[i][j],i,j)
                    if chess_int[i][j]==1:
                        cv2.imshow("chess_piece"+str(i)+str(j),chess[i][j])
            #print(chess_int)
            for i in range(8):
                cv2.rectangle(chess_bord_2,(0,100*i),(800,100*(i+1)),(255,0,0),2)
            for i in range(8):
                cv2.rectangle(chess_bord_2,(100*i,0),(100*(i+1),800),(255,0,0),2)
            cv2.imshow("chess_board2",chess_bord_2)

        
            cv2.rectangle(frame,(b_rect[0],b_rect[1]),(b_rect[0]+b_rect[2],b_rect[1]+b_rect[3]),(0,255,0),2)
        cv2.imshow('frame',frame)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

print(chess_int)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

