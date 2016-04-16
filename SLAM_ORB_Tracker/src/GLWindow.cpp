//
//  GLWindow.cpp
//  OpenGLUI
//
//  Created by Sen Yu on 4/13/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#include "GLWindow.hpp"

GLWindow* GLWindow::pThis = NULL;

GLWindow::GLWindow(std::string name, int w, int h):
    mnWindWidth(w), mnWinHeight(h),
    msTitleName(name),
    mfViewHeight(50),
    mfDrawScale(20),
    posdx(0), posdy(0), posdz(0),
    dirdx(0), dirdy(-1), dirdz(0),
    mbIsDragging(0),

    mfCameraX(0), mfCameraY(0), mfCameraZ(0),
    mfCameraPosDX(0), mfCameraPosDY(0), mfCameraPosDZ(0),
    mfCameraPosNX(mfCameraX), mfCameraPosNY(mfCameraY), mfCameraPosNZ(mfCameraZ),
    mfCameraDirDX(0), mfCameraDirDY(-1), mfCameraDirDZ(0),
    mfDragX(0), mfDragY(0),
    mfDragDX(0), mfDragDY(0),
    meDragMouseButton(0),

    mbSpecialKey(0),
    mpTracker(NULL),
    mpMap(NULL),
    mpMapDrawer(NULL)

{
    pThis = this;
    int fakeArgc = 0;
    glutInit(&fakeArgc, NULL);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    //    glutInitWindowPosition(100, 100);
    glutInitWindowSize(mnWindWidth, mnWinHeight);
    glutCreateWindow(msTitleName.c_str());

    glutReshapeFunc(_funcReshap); // window reshape callback
    glutDisplayFunc(_funcDisplay); // (re)display callback
    glutIdleFunc(_funcIdle); // incremental update
    glutIgnoreKeyRepeat(1); // ignore key repeat when holding key down
    glutMouseFunc(_funcMouse); // process mouse button push/release
    glutMotionFunc(_funcMotion); // process mouse dragging motion
    glutKeyboardFunc(_funcKeyboard); // process standard key clicks
    glutSpecialFunc(_funcSpecial); // process special key pressed
    // Warning: Nonstandard function! Delete if desired.
    glutSpecialUpFunc(_funcSpecialUp); // process special key release
    glDepthFunc(GL_LEQUAL);


}

static float getFrameDirArr[] = {
    -1.0f,-0.5f,0.0f,
    1.0f,-0.5f,0.0f,
    1.0f,0.5f,0.0f,
    -1.0f,0.5f,0.0f
};
static cv::Mat getFrameDirMat(4,3,CV_32FC1, getFrameDirArr);


void drawFrameOnce(shared_ptr<FrameState> pF , float mfDrawScale, cv::Scalar cPlane = cv::Scalar(0,0,0) , cv::Scalar cLine = cv::Scalar(0,0,0) ) {

    PoseState pose;

    cv::Mat mT2w = pF->mT2w;
    cv::Mat mMatR = pF->mMatR;
    cv::Mat mO2w = pF->mO2w;

    cv::Mat frameDir = getFrameDirMat * mT2w.rowRange(0,3).colRange(0,3);


    pose.mPos = Utils::convertToPoint3d( mO2w  );
    pose.mDir = Utils::convertToPoint3d( mMatR.row(2)  );


    glPushMatrix();
        glTranslatef(pose.mPos.x * mfDrawScale, pose.mPos.y * mfDrawScale, pose.mPos.z * mfDrawScale);
        glColor3f( cPlane[0] / 255.0f, cPlane[1] / 255.0f, cPlane[2] / 255.0f ); // Let it be blue
        glBegin(GL_QUADS); // 2x2 pixels
            for(int i=0;i<4;i++) {
                glVertex3f( frameDir.at<float>(i,0),
                    frameDir.at<float>(i,1),
                    frameDir.at<float>(i,2));
            }

        glEnd();
        glColor3f( cLine[0] / 255.0f, cLine[1] / 255.0f, cLine[2] / 255.0f ); // Let it be blue
        glBegin(GL_LINE_STRIP);
            glLineWidth(2.0);
            for(int i=0;i<4;i++) {
                glVertex3f( frameDir.at<float>(i,0),
                    frameDir.at<float>(i,1),
                    frameDir.at<float>(i,2));
            }
        glEnd();
    glPopMatrix();


}

void drawPointOnce( cv::Mat mPos, float mfDrawScale ) {
    cv::Point3f pos = Utils::convertToPoint3d(mPos);

    glVertex3f(pos.x * mfDrawScale, pos.y * mfDrawScale, pos.z * mfDrawScale);

}

void GLWindow::drawAllElement() {


    std::set<shared_ptr<FrameState>>& spF = mpMapDrawer->cacheRefGetAllSetFrame();

    for(auto iter=spF.begin();iter!=spF.end();iter++) {
        shared_ptr<FrameState> pF = *iter;
        if ( !pF ) continue;
        drawFrameOnce(pF ,mfDrawScale, cv::Scalar(128,128,255), cv::Scalar(128,128,255) );
    }
    std::set<shared_ptr<KeyFrameState>>& spKF = mpMap->cacheRefGetAllSetKeyFrame();
    for(auto iter=spKF.begin();iter!=spKF.end();iter++) {
        shared_ptr<KeyFrameState> pKF = *iter;
        if ( !pKF ) continue;
        drawFrameOnce(pKF->mpFrame,mfDrawScale, cv::Scalar(0,128,0), cv::Scalar(0,0,0) );
    }

    std::vector<shared_ptr<MapPoint>>& vpLMP = mpTracker->cacheRefGetAllVectorLocalMapPoint();
    std::set<shared_ptr<MapPoint>>&  spMP = mpMap->cacheRefGetAllSetMapPoint();
    glPushMatrix();

    glPointSize(1.5f);
    glColor3f( 0,0,0 ); // Let it be blue
    glBegin(GL_POINTS);
    for(auto iter=spMP.begin();iter!=spMP.end();iter++) {

        shared_ptr<MapPoint> pMP = *iter;
        if ( !pMP ) continue;
        if ( pMP->isBad() )
            continue;
        //if ( spLMP.count(pMP) ) continue;
        // for using of mPos, should lock it or maybe Work-thread write to it and CRASH.
        pMP->lockMutexMPos(0);
            drawPointOnce( pMP->getMPosRef() , mfDrawScale );
        pMP->lockMutexMPos(1);

    }
    glEnd();


    glPointSize(2.0f);
    glColor3f( 255,0,0 ); // Let it be blue
    glBegin(GL_POINTS);
    for(auto iter=vpLMP.begin();iter!=vpLMP.end();iter++) {

        shared_ptr<MapPoint> pMP = *iter;
        if (!pMP) continue;
        if (pMP->isBad())
            continue;
        // for using of mPos, should lock it or maybe Work-thread write to it and CRASH.
        pMP->lockMutexMPos(0);
            drawPointOnce( pMP->getMPosRef() , mfDrawScale );
        pMP->lockMutexMPos(1);

    }
    glEnd();
    glPopMatrix();

}

void GLWindow::onReshape(int nw, int nh)
{

    printf("Reshap w %d h %d\n",nw,nh);
    mnWindWidth = nw;
    mnWinHeight = nh;
    float ratio =  ((float) mnWindWidth) / ((float) mnWinHeight); // window aspect ratio
    glMatrixMode(GL_PROJECTION); // projection matrix is active
    glLoadIdentity(); // reset the projection
    gluPerspective(300.0, ratio, 0.01, 1000.0); // perspective transformation
    glMatrixMode(GL_MODELVIEW); // return to modelview mode
    glViewport(0, 0, mnWindWidth, mnWinHeight); // set viewport (drawing area) to entire

}


void GLWindow::onIdle(void)
{
    onDisplay();
    //glutPostRedisplay(); // redisplay everything
}


void GLWindow::onDisplay(void)
{
    int i, j;

    // Clear color and depth buffers
    glClearColor(0.9,0.9,0.9, 1.0); // sky color is light blue
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    // Reset transformations
    glLoadIdentity();

    // Set the camera centered at (x,y,1) and looking along directional
    // vector (lx, ly, 0), with the z-axis pointing up
    moveCamera();

    gluLookAt(

        mfCameraX - posdx - dirdx * mfViewHeight,
        mfCameraY + posdy + dirdy * mfViewHeight,
        mfCameraZ + posdz + dirdz * mfViewHeight,

        mfCameraX - posdx,
        mfCameraY + posdy ,
        mfCameraZ + posdz,

        -dirdx,
        0.0,
        -0.7);

    drawCoord();

    drawAllElement();

    glutSwapBuffers(); // Make it all visible
}

void GLWindow::onKeyBoard(unsigned char key, int xx, int yy)
{
    if (key == ESC || key == 'q' || key == 'Q') exit(0);
}

void GLWindow::onSpecial(int key, int xx, int yy)
{
    switch (key) {
        case GLUT_KEY_SHIFT_L :
        case GLUT_KEY_SHIFT_R : mbSpecialKey = 1; break;
    }
}

void GLWindow::onSpecialUp(int key, int x, int y)
{
    switch (key) {
        case GLUT_KEY_SHIFT_L :
        case GLUT_KEY_SHIFT_R : mbSpecialKey = 0; break;
    }
}

void GLWindow::onMotion(int x, int y)
{

    if (mbIsDragging) { // only when dragging
        if ( meDragMouseButton == GLUT_RIGHT_BUTTON ) {

            mfDragDX = (x - mfDragX) * 0.1 ;
            mfDragDY = (y - mfDragY) * 0.1;

            posdx = (mfDragDX + mfCameraPosDX);
            posdy = ( mfCameraPosDY);
            posdz = (mfDragDY + mfCameraPosDZ);


            printf("moveing x %d y %d dargX %f mfDragY %f cameraPosDX %f, camedaDY %f, mfDragDX %f mfDragDY %f\n",x,y, mfDragX, mfDragY, mfCameraPosDX, mfCameraPosDY, mfDragDX, mfDragDY);
        }
        else if ( meDragMouseButton == GLUT_LEFT_BUTTON ) {
            mfDragDX = (x - mfDragX) * 0.01;
            mfDragDY = (y - mfDragY) * 0.01;



            dirdx = (mfDragDX + mfCameraDirDX);
            dirdy = ( mfCameraDirDY);
            dirdz = (mfDragDY + mfCameraDirDZ );

            float normal = sqrt(dirdx*dirdx + dirdy*dirdy+dirdz*dirdz);
            dirdx/= normal;
            dirdy/= normal;
            dirdz/= normal;


            dirdy = fabs(dirdy) < 0.1 ? copysignf(0.1, dirdy) : dirdy;
        }

    }
}

void GLWindow::onMouse(int button, int state, int x, int y)
{
    if ( button == 3 ) {

        mfViewHeight -= 0.1;
    }
    else if ( button == 4 ) {

        mfViewHeight += 0.1;
    }

    if (
        button == GLUT_LEFT_BUTTON ||
            button == GLUT_RIGHT_BUTTON) {


        if (state == GLUT_DOWN && mbIsDragging == 0) { // left mouse button pressed
            meDragMouseButton = button;
            mbIsDragging = 1; // start dragging
            mfDragX = x; // save x where button first pressed
            mfDragY = y; // save x where button first pressed

        }
        else if ( state == GLUT_UP && mbIsDragging == 1) { /* (state = GLUT_UP) */

            mbIsDragging = 0; // no longer dragging
            if ( button == GLUT_RIGHT_BUTTON ) {
                mfCameraPosDX = posdx; // update camera turning angle
                mfCameraPosDY = posdy; // update camera turning angle
                mfCameraPosDZ = posdz; // update camera turning angle
            }
            else if ( button == GLUT_LEFT_BUTTON ) {
                mfCameraDirDX = dirdx;
                mfCameraDirDY = dirdy;
                mfCameraDirDZ = dirdz;
            }
            mfDragDX = 0;
            mfDragDY = 0;

        }

    }
}


void GLWindow::moveCamera() {
    boost::mutex::scoped_lock lock(mMutexUpdateCameraPos);
    float speed = 0.1f;
    float dx = (mfCameraPosNX - mfCameraX);
    float dy = (mfCameraPosNY - mfCameraY);
    float dz = (mfCameraPosNZ - mfCameraZ);
    float normal = sqrt(dx*dx+dy*dy+dz*dz);
    if (  normal > 1.0f ) {
        dx /= normal;
        dy /= normal;
        dz /= normal;
    }
    mfCameraX = mfCameraX + speed * dx;
    mfCameraY = mfCameraY + speed * dy;
    mfCameraZ = mfCameraZ + speed * dz;
}
void GLWindow::setNewCameraPos(float _nx, float _ny, float _nz) {
    boost::mutex::scoped_lock lock(mMutexUpdateCameraPos);
    mfCameraPosNX = _nx * mfDrawScale;
    mfCameraPosNY = _ny * mfDrawScale;
    mfCameraPosNZ = _nz * mfDrawScale;
}


void GLWindow::loopOnce() {
    glutPostRedisplay();
    glutMainLoopEvent();
}




void GLWindow::drawCoord(int step , float limit) {
    float drawIter;
    glColor3f(0.4,0.4,0.4);
    glEnable (GL_LINE_SMOOTH);

    glEnable (GL_BLEND);//启用混合
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    /*对图像质量和渲染速度之间控制权衡关系*/

    for(drawIter = step; drawIter < limit; drawIter += step) {

        for(int bit = -1;bit<=1;bit+=2) {

            glBegin(GL_LINES);
            glLineWidth(0.5);
            glVertex3f(drawIter * bit,0.0f,-limit);
            glVertex3f(drawIter * bit,0.0f,limit);
            glEnd();
        }
    }
    for(drawIter = step; drawIter < limit; drawIter += step) {

        for(int bit = -1;bit<=1;bit+=2) {

            glBegin(GL_LINES);
            glLineWidth(0.5);
            glVertex3f(-limit,0.0f, drawIter * bit);
            glVertex3f(limit,0.0f, drawIter * bit);
            glEnd();
        }
    }
    // x-axis
    glColor3f(1.0,0,0);
    glBegin(GL_LINES);
    glLineWidth(5);
    //glVertex3f(0,-limit,0.0f);
    glVertex3f(0,0,0.0f);
    glVertex3f(limit,0,0.0f);
    glEnd();

    // y-axis
    glColor3f(0,1.0,0);
    glBegin(GL_LINES);
    glLineWidth(5);
    //glVertex3f(-limit,0,0.0f);
    glVertex3f(0,0,0.0f);
    glVertex3f(0,limit,0.0f);
    glEnd();

    // z-axis
    glColor3f(0,0,1.0);
    glBegin(GL_LINES);
    glLineWidth(5);
    //glVertex3f(0,0.0f,-limit);
    glVertex3f(0,0.0f,0);
    glVertex3f(0,0.0f,limit);
    glEnd();


}

void GLWindow::setMap(Map *_p) {
    mpMap = _p;
}

void GLWindow::setMapDrawer(MapDrawer *_p) {
    mpMapDrawer = _p;
}

void GLWindow::setTracker(Tracker *_p) {
    mpTracker = _p;
}
