//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include <vector>
#include <string>
#include <iostream>
#include <time.h>
#include "dollar.h"

using namespace std;

class Side
{
	public:
	Joint Shoulder;
	Joint ShoulderLast;
	CameraSpacePoint ShoulderCenter;
	Joint Hand;
	Joint HandLast;
	HandState State;
	HandState StateLast;
	DWORD Count;
};

class Action
{
	public:
	enum ActionType
	{
		Key_VK,
		Mouse,
		Command
	};

	ActionType type;
	string action;
	WORD vk;
	int mouse;
	int wheelamount;
};

class CBodyBasics
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CBodyBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CBodyBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;

    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;

    // Body reader
    IBodyFrameReader*       m_pBodyFrameReader;

    // Direct2D
    ID2D1Factory*           m_pD2DFactory;

    // Body/hand drawing
    ID2D1HwndRenderTarget*  m_pRenderTarget;
    ID2D1SolidColorBrush*   m_pBrushJointTracked;
    ID2D1SolidColorBrush*   m_pBrushJointInferred;
    ID2D1SolidColorBrush*   m_pBrushBoneTracked;
    ID2D1SolidColorBrush*   m_pBrushBoneInferred;
    ID2D1SolidColorBrush*   m_pBrushHandClosed;
    ID2D1SolidColorBrush*   m_pBrushHandOpen;
    ID2D1SolidColorBrush*   m_pBrushHandLasso;

    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();
    
    /// <summary>
    /// Handle new body data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="nBodyCount">body data count</param>
    /// <param name="ppBodies">body data in frame</param>
    /// </summary>
    void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

    /// <summary>
    /// Ensure necessary Direct2d resources are created
    /// </summary>
    /// <returns>S_OK if successful, otherwise an error code</returns>
    HRESULT EnsureDirect2DResources();

    /// <summary>
    /// Dispose Direct2d resources 
    /// </summary>
    void DiscardDirect2DResources();

    /// <summary>
    /// Converts a body point to screen space
    /// </summary>
    /// <param name="bodyPoint">body point to tranform</param>
    /// <param name="width">width (in pixels) of output buffer</param>
    /// <param name="height">height (in pixels) of output buffer</param>
    /// <returns>point in screen-space</returns>
    D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);

    /// <summary>
    /// Draws a body 
    /// </summary>
    /// <param name="pJoints">joint data</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    void                    DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);

    /// <summary>
    /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
    /// </summary>
    /// <param name="handState">state of the hand</param>
    /// <param name="handPosition">position of the hand</param>
    void                    DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

    /// <summary>
    /// Draws one bone of a body (joint to joint)
    /// </summary>
    /// <param name="pJoints">joint data</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="joint0">one joint of the bone to draw</param>
    /// <param name="joint1">other joint of the bone to draw</param>
    void                    DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);

		// Added by Liu Jiaxin
			private:

			const double ljx_c_dDistanceThreshold = 0.05;
			const double ljx_c_dMonitorWidth = 0.96;
			const double ljx_c_dMonitorHeight = 0.54;

			Side ljx_m_sLeft;
			Side ljx_m_sRight;

			bool              ljx_m_bRecording;
			FILE             *ljx_m_pFile;
			int               ljx_m_nCount;
			int mx = 65535 / 2;
			int my = 65535 / 2;
			char name[255];
			ID2D1HwndRenderTarget*  m_pTempRenderTarget;
			ID2D1Factory* m_pTempFactory;
			vector<D2D1_POINT_2F> ljx_m_vNewTemp;
			vector<VEC> ljx_m_vTempShowing;
			ID2D1SolidColorBrush* BrushBlack;
			ID2D1SolidColorBrush* BrushRed;
			ID2D1SolidColorBrush* BrushBlue;
			ID2D1SolidColorBrush* BrushGreen;

			vector<Joint> pdrawlist;
			vector<ID2D1SolidColorBrush*> brushlist;
			int p2draw;

			public:
			void   ljxProcessGesture(Joint *joints, HandState hsLeft, HandState hsRight);
			void   ljxCalShoulderPos(Joint Shoulder, Joint Hand, HandState state, Side &side);
			void   ljxResponse();
			double ljxJointDistance(Joint p1, Joint p2);

			void ljxStartRecording();
			void ljxEndRecording();
			void ljxWriteRecord(Joint RightShoulder, Joint RightHand, HandState hsRightHand);

			void ljxInitialize();
			void ljxStartRecognize();
			void ljxEndRecognize();
			void ljxRecognize(Joint joint);
			void ljxStartDrawTemp();
			void ljxDrawTemplate(int x, int y, int width, int height);
			void ljxEndDrawTemp();
			void ljxShowJoint();
			void ljxShowTemplate();
};