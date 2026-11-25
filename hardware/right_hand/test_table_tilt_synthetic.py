"""Motor-free image test of board/marker tilt estimation against known geometry."""

from pathlib import Path

import cv2 as cv
import numpy as np

from sac_table_tilt_from_photo import analyze

ROOT = Path(__file__).resolve().parent


def main():
    with np.load(ROOT/'camera1_720.npz') as f:
        K, dist=f['K'].copy(),f['dist'].copy()
    output=ROOT/'table_tilt_synthetic_validation'
    output.mkdir(exist_ok=True)
    square=.024
    cell=80
    board=np.full((7*cell,10*cell),255,np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y)%2==0:
                board[y*cell:(y+1)*cell,x*cell:(x+1)*cell]=0
    board_corners=np.asarray([[-square,-square,0],[9*square,-square,0],
                              [9*square,6*square,0],[-square,6*square,0]],np.float32)
    board_img,_=cv.projectPoints(board_corners,np.zeros(3),
                                  np.array([-.12,-.06,.48]),K,dist)
    board_img=board_img.reshape(4,2).astype(np.float32)
    image=np.full((720,1280),200,np.uint8)
    src=np.asarray([[0,0],[board.shape[1]-1,0],[board.shape[1]-1,board.shape[0]-1],
                    [0,board.shape[0]-1]],np.float32)
    H=cv.getPerspectiveTransform(src,board_img)
    warped=cv.warpPerspective(board,H,(1280,720),borderValue=200)
    mask=cv.warpPerspective(np.full_like(board,255),H,(1280,720))
    image[mask>0]=warped[mask>0]
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    marker=cv.aruco.generateImageMarker(dictionary,40,600)
    marker=cv.copyMakeBorder(marker,100,100,100,100,cv.BORDER_CONSTANT,value=255)
    side=.0282/2
    marker_obj=np.asarray([[-side,side,0],[side,side,0],
                           [side,-side,0],[-side,-side,0]],np.float32)
    R_x=np.array([[1,0,0],[0,np.cos(np.radians(20)),-np.sin(np.radians(20))],
                  [0,np.sin(np.radians(20)),np.cos(np.radians(20))]])
    R_y=np.array([[np.cos(np.radians(10)),0,np.sin(np.radians(10))],
                  [0,1,0],[-np.sin(np.radians(10)),0,np.cos(np.radians(10))]])
    rotation=R_y@R_x
    rv,_=cv.Rodrigues(rotation)
    # Image top points toward negative camera Y. Render the generated marker
    # without mirroring its bit pattern; the analyzer uses detected corner order.
    outer_obj=marker_obj[[3,2,1,0]]*(800/600)
    marker_img,_=cv.projectPoints(outer_obj,rv,np.array([.10,.02,.30]),K,dist)
    marker_img=marker_img.reshape(4,2).astype(np.float32)
    src_marker=np.asarray([[0,0],[799,0],[799,799],[0,799]],np.float32)
    Hm=cv.getPerspectiveTransform(src_marker,marker_img)
    warped_marker=cv.warpPerspective(marker,Hm,(1280,720),borderValue=200)
    marker_mask=cv.warpPerspective(np.full_like(marker,255),Hm,(1280,720))
    image[marker_mask>0]=warped_marker[marker_mask>0]
    color=cv.cvtColor(image,cv.COLOR_GRAY2BGR)
    for index in range(5):
        (output/f'frame_{index}.png').write_bytes(cv.imencode('.png',color)[1].tobytes())
    found,_=cv.findChessboardCornersSB(image,(9,6),flags=cv.CALIB_CB_NORMALIZE_IMAGE)
    detector=cv.aruco.ArucoDetector(dictionary,cv.aruco.DetectorParameters())
    _,ids,_=detector.detectMarkers(image)
    print('Synthetic detections:',bool(found),None if ids is None else ids.ravel().tolist())
    result=analyze(output)
    assert result['frames_used']==5, result
    assert 15<result['marker_table_angle_deg']<30, result
    print('Synthetic board + ID40 tilt passed:',result['marker_table_angle_deg'])


if __name__=='__main__':
    main()
