//
//  GLWindow.hpp
//  OpenGLUI
//
//  Created by Sen Yu on 4/13/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef GLWindow_hpp
#define GLWindow_hpp

#include "stdafx.hpp"

#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GL/freeglut.h>

#include "MapDrawer.hpp"
#include "Map.hpp"
#include "Tracker.hpp"

#define ESC 27

class MapDrawer;
class Tracker;

class GLWindow {
private:
    typedef boost::interprocess::interprocess_semaphore semaphore;

    static GLWindow* pThis;

    int mnWindWidth, mnWinHeight;

    int mbSpecialKey;
    int mbIsDragging;
    int meDragMouseButton;

    float mfViewHeight;
    float mfCameraX, mfCameraY, mfCameraZ;
    float mfCameraPosDX, mfCameraPosDY, mfCameraPosDZ;
    float mfCameraDirDX, mfCameraDirDY, mfCameraDirDZ;
    float mfDragX, mfDragY;
    float mfDragDX, mfDragDY;

    float posdx, posdy, posdz;
    float dirdx, dirdy, dirdz;
    float mfCameraPosNX, mfCameraPosNY, mfCameraPosNZ;

    float mfDrawScale ;
    MapDrawer* mpMapDrawer;
    Map * mpMap;
    Tracker* mpTracker;
    const std::string msTitleName;

    boost::mutex mMutexUpdateCameraPos;

public:
    GLWindow(std::string name = "GLWindow", int w = 800, int h = 600);

private:
    static void _funcReshap(int x, int y) {
        pThis->onReshape(x,y);
    }
    static void _funcDisplay() {
        pThis->onDisplay();
    }
    static void _funcIdle() {
        pThis->onIdle();
    }
    static void _funcMouse(int button, int state, int x, int y) {
        pThis->onMouse(button, state, x, y);
    }
    static void _funcMotion(int x, int y) {
        pThis->onMotion(x,y);
    }
    static void _funcKeyboard(unsigned char key, int xx, int yy) {
        pThis->onKeyBoard(key,xx,yy);
    }
    static void _funcSpecial(int key, int xx, int yy) {
        pThis->onSpecial(key,xx,yy);
    }
    static void _funcSpecialUp(int key, int xx, int yy) {
        pThis->onSpecialUp(key,xx,yy);
    }


public:
    void moveCamera();

    void setNewCameraPos(float _nx, float _ny, float _nz);

    void loopOnce();

    void drawCoord(int step = 10, float limit = 1e3);

    void setMap(Map* _p);

    void setMapDrawer(MapDrawer* _p);

    void setTracker(Tracker* _p);

public:

    virtual void drawAllElement();

    virtual void onReshape(int nw, int nh);

    virtual void onIdle(void);

    virtual void onDisplay(void);

    virtual void onKeyBoard(unsigned char key, int xx, int yy);

    virtual void onSpecial(int key, int xx, int yy);

    virtual void onSpecialUp(int key, int x, int y);

    virtual void onMotion(int x, int y);

    virtual void onMouse(int button, int state, int x, int y);



};


#endif /* GLWindow_hpp */
