//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include <math.h>
#include "filter.h"
#include <process.h>
#include <stdlib.h>
#include <Shellapi.h>
//#include "hmm.h"
//#include "hmmkinect.h"

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

vector<Action> actionlist;
int actionNum = 0;
bool sendinput = false;
bool drawnewtemp = false;
bool drawstarted = false;
bool showtemp = false;
bool record = false;
bool recording = false;
string nowconfig = "";
HACCEL hAccel;
wstring nowtemplate = L"";
//HMM hmm;
//int hmm_temp_count = 0;
//vector<Sequence> hmm_templates;
//const int T = 32;

bool loadtemplate(int num, string filename)
{
	fstream in(filename);
	if (in.fail())
	{
		return false;
	}
  //JOINTS tem;
  //JOINT joint;
  //Sequence seq;
	vector<VEC> v;
	VEC t;
	while (in >> t)
	{
		v.push_back(t);
    //joint.Position.X = t.x;
    //joint.Position.Y = t.y;
    //tem.push_back(joint);
	}
  //tem.Normalize();
  //tem.Resample(T+1);
  //SequenceGen(tem, seq);
  //hmm_temp_count++;
  //hmm_templates.push_back(seq);
	v = normalize(v);
	templates.push_back(v);
	return true;
}

void loadConfig(wstring filename, wstring path)
{
	templates.clear();
	actionlist.clear();
  //hmm_templates.clear();
  //hmm_temp_count = 0;
	string spath(path.begin(), path.end());

	fstream config(filename, ios::in);
	if (config.fail())
	{
		MessageBoxA(NULL, "No such config file exist!", "Error", MB_OK);
    return;
	}
	while (true)
	{
		string command = "";
		string tempname = "";
		string action = "";
		config >> command;
		if (config.eof()) break;
		if (strcmpi(command.c_str(), "TEMPLATE") == 0)
		{
			config >> tempname;
			tempname = spath + tempname;
			if (config.eof()) break;
			if (!loadtemplate(actionNum, tempname))
			{
				string msg = "Template file " + tempname + " not exist!";
				MessageBoxA(NULL, msg.c_str(), "Error!", MB_OK);
				templates.clear();
				actionlist.clear();
				return;
			}
			config >> command;
			if (config.eof()) break;
			Action newaction;
			if (strcmpi(command.c_str(), "ACTION") == 0)
			{
				getline(config, action);
				newaction.type = Action::Command;
				newaction.action = action;
			}
			else if (strcmpi(command.c_str(), "MOUSE") == 0)
			{
				config >> command;
				if (strcmpi(command.c_str(), "WHEEL") == 0)
				{
					newaction.mouse = MOUSEEVENTF_WHEEL;
					newaction.wheelamount = 0;
					config >> newaction.wheelamount;
				}
				else if (strcmpi(command.c_str(), "RCLK") == 0)
				{
					newaction.mouse = MOUSEEVENTF_RIGHTDOWN | MOUSEEVENTF_RIGHTUP;
				}
				else
				{
					MessageBoxA(NULL, "Unrecognized command!", "Error", MB_OK);
					templates.clear();
					actionlist.clear();
				}
				newaction.type = Action::Mouse;
			}
			else if (strcmpi(command.c_str(), "VK") == 0)
			{
				config >> newaction.vk;
				newaction.type = Action::Key_VK;
			}
			actionNum++;
			actionlist.push_back(newaction);
		}
	}
}

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_pD2DFactory(NULL),
		m_pTempFactory(NULL),
    m_pRenderTarget(NULL),
		m_pTempRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL),
    m_pBrushHandClosed(NULL),
    m_pBrushHandOpen(NULL),
    m_pBrushHandLasso(NULL),
		p2draw(0)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

		ZeroMemory(&ljx_m_sLeft, sizeof(ljx_m_sLeft));
		ljx_m_sLeft.State = HandState_NotTracked;
		ljx_m_sLeft.StateLast = HandState_NotTracked;

		ZeroMemory(&ljx_m_sRight, sizeof(ljx_m_sRight));
		ljx_m_sRight.State = HandState_NotTracked;
		ljx_m_sRight.StateLast = HandState_NotTracked;
		ljx_m_bRecording = false;
}
  

/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);
		SafeRelease(m_pTempFactory);

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

		int argc;
		LPWSTR *arglist;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CBodyBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);
		hAccel = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDR_ACCELERATOR1));
		arglist = CommandLineToArgvW(GetCommandLine(), &argc);
		if (argc >= 1)
		{
			for (int i = 1; i < __argc; i++)
			{
				wstring cmd(arglist[i]);
				if (cmd == L"-input")
				{
					sendinput = true;
				}
				else if (cmd == L"-config")
				{
					wstring filename;
					if (i + 1 <= argc)
					{
						filename = arglist[i + 1];
						i++;
						int pos = filename.length();
						while (filename[pos] != wchar_t('\\')) pos--;
						wstring fileparent(L"");
						for (int l = 0; l <= pos; l++)
						{
							fileparent = fileparent + filename[l];
						}
						loadConfig(filename, fileparent);
					}
					else
					{
						MessageBoxA(NULL, "No config file specified!", "Error", MB_OK);
						exit(0);
					}
				}
				else
				{
					MessageBox(NULL, (wstring(L"Unknown parameter : ") + cmd).c_str(), L"Error", MB_OK);
				}
			}
		}

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }
						if (!TranslateAccelerator(m_hWnd, hAccel, &msg))
						{
							TranslateMessage(&msg);
							DispatchMessageW(&msg);
						}
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyBasics::Update()
{
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = {0};

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nTime, BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);
						D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pTempFactory);

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

				case WM_COMMAND:
					if (drawnewtemp)
					{
						if (drawstarted)
						{
							drawstarted = false;
							ljxEndDrawTemp();
						}
						drawnewtemp = false;
					}
					switch (LOWORD(wParam))
					{
						case IDC_RECORD:
						{
							if (!record)
							{
							 SetWindowText(GetDlgItem(m_hWnd, IDC_RECORD), L"Disable Record");
							 record = true;
							 recording = false;
							}
							else
							{
							 if (recording)
							 {
								 ljxEndRecording();
							 }
							 SetWindowText(GetDlgItem(m_hWnd, IDC_RECORD), L"Enable Record");
							 record = false;
							 recording = false;
							}
							
							break;
						}

						case IDC_CONFIG:
						{
							IFileDialog *pfd;
							HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_INPROC_SERVER,
																					 IID_PPV_ARGS(&pfd));
							DWORD dwFlags;
							hr = pfd->GetOptions(&dwFlags);
							hr = pfd->SetOptions(dwFlags | FOS_FORCEFILESYSTEM);
							COMDLG_FILTERSPEC filter[] = {
							 { L"Config File", L"*.txt" },
							 { L"All File", L"*.*" }
							};
							hr = pfd->SetFileTypes(2, filter);
							hr = pfd->SetFileTypeIndex(1);
							hr = pfd->Show(m_hWnd);
							if (hr == S_OK)
							{
								IShellItem *result;
								hr = pfd->GetResult(&result);
								LPWSTR filePath;
								hr = result->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &filePath);
								int pos = lstrlen(filePath);
								while (filePath[pos] != wchar_t('\\')) pos--;
								wstring fileparent(L"");
								for (int i = 0; i <= pos; i++)
								{
									fileparent = fileparent + filePath[i];
								}
								loadConfig(wstring(filePath), fileparent);
							}
							break;
						}
						case IDC_CHANGE:
							sendinput = !sendinput;
							break;
						case IDC_TEMPLATE:
						{
							if (showtemp)
							{
								SetWindowText(GetDlgItem(m_hWnd, IDC_TEMPLATE), L"Open Template");
							  showtemp = false;
							  ljx_m_vTempShowing.clear();
							  break;
							}
							IFileDialog *pfd;
							HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_INPROC_SERVER,
																					 IID_PPV_ARGS(&pfd));
							DWORD dwFlags;
							hr = pfd->GetOptions(&dwFlags);
							hr = pfd->SetOptions(dwFlags | FOS_FORCEFILESYSTEM);
							COMDLG_FILTERSPEC filter[] = {
							 { L"All File", L"*.*" }
							};
							hr = pfd->SetFileTypes(1, filter);
							hr = pfd->SetFileTypeIndex(1);
							hr = pfd->Show(m_hWnd);
							if (hr == S_OK)
							{
							 IShellItem *result;
							 hr = pfd->GetResult(&result);
							 LPWSTR fileName;
							 LPWSTR filePath;
							 hr = result->GetDisplayName(SIGDN_NORMALDISPLAY, &fileName);
							 hr = result->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &filePath);
							 fstream temp(filePath, ios::in);
							 ljx_m_vTempShowing.clear();
							 if (!temp)
							 {
								 MessageBoxA(NULL, "No such file!", "Error", MB_OK);
							 }
							 VEC t;
							 while (temp >> t) ljx_m_vTempShowing.push_back(t);
							 showtemp = true;
							 SetWindowText(GetDlgItem(m_hWnd, IDC_TEMPLATE), L"Close Template");
							}
							break;
						}
						case IDC_NEWTEMP:
							showtemp = false;
							SetWindowText(GetDlgItem(m_hWnd, IDC_TEMPLATE), L"Open Template");
							ljx_m_vTempShowing.clear();
							drawnewtemp = true;
							break;
					}
					break;
				case WM_LBUTTONDOWN:
				{
					if (!drawnewtemp)
					{
					 break;
					}
					RECT rct;
					GetWindowRect(GetDlgItem(m_hWnd, IDC_TEMPVIEW), &rct);

					POINT lefttop;
					lefttop.x = rct.left; lefttop.y = rct.top;
					ScreenToClient(m_hWnd, &lefttop);
					POINT rightbottom;
					rightbottom.x = rct.right; rightbottom.y = rct.bottom;
					ScreenToClient(m_hWnd, &rightbottom);

					int x, y;
					x = LOWORD(lParam);
					y = HIWORD(lParam);
					if (x >= lefttop.x && x <= rightbottom.x && y >= lefttop.y && y <= rightbottom.y)
					{
						ljxStartDrawTemp();
						drawstarted = true;
					}
					break;
				}
				case WM_MOUSEMOVE:
				{
					if (!drawnewtemp)
					{
					 break;
					}
					RECT rct;
					GetWindowRect(GetDlgItem(m_hWnd, IDC_TEMPVIEW), &rct);

					POINT lefttop;
					lefttop.x = rct.left; lefttop.y = rct.top;
					ScreenToClient(m_hWnd, &lefttop);
					POINT rightbottom;
					rightbottom.x = rct.right; rightbottom.y = rct.bottom;
					ScreenToClient(m_hWnd, &rightbottom);

					int x, y;
					x = LOWORD(lParam);
					y = HIWORD(lParam);
					if (x >= lefttop.x && x <= rightbottom.x && y >= lefttop.y && y <= rightbottom.y && drawstarted)
					{
						x -= lefttop.x; y -= lefttop.y;
						int width = rightbottom.x - lefttop.x;
						int height = rightbottom.y - lefttop.y;
						ljxDrawTemplate(x, y, width, height);
					}
					break;

				}
				case WM_LBUTTONUP:
				{
				  if (drawnewtemp)
					{
						drawstarted = false;
						ljxEndDrawTemp();
					}
					drawnewtemp = false;
					break;
				}

    }

    return FALSE;
}


/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
    if (m_hWnd)
    {
        HRESULT hr = EnsureDirect2DResources();

        if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
        {
					ljxShowJoint();
					ljxShowTemplate();
            m_pRenderTarget->BeginDraw();
            m_pRenderTarget->Clear();

            RECT rct, PointRect;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
            int width = rct.right;
            int height = rct.bottom;
						int mintracted = 9999999;

            for (int i = 0; i < nBodyCount; ++i)
            {
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked && i<=mintracted)
                    {
											mintracted = i;
                        Joint joints[JointType_Count]; 
                        D2D1_POINT_2F jointPoints[JointType_Count];
                        HandState leftHandState = HandState_Unknown;
                        HandState rightHandState = HandState_Unknown;

                        pBody->get_HandLeftState(&leftHandState);
                        pBody->get_HandRightState(&rightHandState);

                        hr = pBody->GetJoints(_countof(joints), joints);
                        if (SUCCEEDED(hr))
                        {
                            for (int j = 0; j < _countof(joints); ++j)
                            {
                                jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
                            }
														//将得到的数据传递给识别模块
														ljxProcessGesture(joints, leftHandState, rightHandState);
                            DrawBody(joints, jointPoints);

                            DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
                            DrawHand(rightHandState, jointPoints[JointType_HandRight]);
                        }
                    }
                }
            }

            hr = m_pRenderTarget->EndDraw();

            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }

        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[255];
				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d %s %s %s",
												fps, (nTime - m_nStartTime), (sendinput) ?L"Inputing" : L"Not Inputing",
												(ljx_m_bRecording)?L"Recognizing" : L"Not Recognizing",
												(recording)?L"Recording":L"Not Recording");
        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
        );

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);

				ljxInitialize();
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);
		SafeRelease(m_pTempRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);

    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = {0};
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

    // Torso
    DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
    DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
    DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);
    
    // Right Arm    
    DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

    // Left Arm
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

    // Right Leg
    DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
    DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
    DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

    // Left Leg
    DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
    DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
    DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CBodyBasics::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
        case HandState_Closed:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
            break;

        case HandState_Open:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
            break;

        case HandState_Lasso:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
            break;
    }
}


//Added by Liu Jiaxin
//识别模块总接口
void CBodyBasics::ljxProcessGesture(Joint *joints, HandState hsLeft, HandState hsRight)
//                                  身体节点数组     左手状态            右手状态
{
	//滤波
	//手滤波
	static Filter RightHandFilter;
	static Filter LeftHandFilter;
	static bool set = false;

	Joint Right_Median  = RightHandFilter.Filter_Median(joints[JointType_HandRight]); //中位数滤波
	Joint Right_Average = RightHandFilter.Filter_Average(Right_Median);              //带权均值滤波
	if (!set)
	{
		Matrix C(1, 4);  C.at(0, 0) = 1;
		Matrix vx(1, 1); vx.at(0, 0) = 0.02;
		Matrix vy(1, 1); vy.at(0, 0) = 0.02;
		Matrix ex(4, 4); ex.at(0, 0) = 0.02; ex.at(1, 1) = 0.02; ex.at(2, 2) = 0.02; ex.at(3, 3) = 0.02;
		Matrix ey(4, 4); ey.at(0, 0) = 0.02; ey.at(1, 1) = 0.02; ey.at(2, 2) = 0.02; ey.at(3, 3) = 0.02;
		RightHandFilter.Kalman_Set(C, vx, vy, ex, ey);
		RightHandFilter.LeastSquareInit(4, 2);
		LeftHandFilter.LeastSquareInit(4, 2);
		set = true;
	}
	Joint Right_Kalman = RightHandFilter.Filter_Kalman(Right_Median);
	Joint Right_LS = RightHandFilter.Filter_LeastSquare(Right_Median);
	Joint Right_Particle = RightHandFilter.Filter_Particle(Right_Median);

	Joint Left_Median  = LeftHandFilter.Filter_Median(joints[JointType_HandLeft]);    //中位数滤波
	Joint Left_LS = LeftHandFilter.Filter_LeastSquare(joints[JointType_HandLeft]);
	Joint Left_Average = LeftHandFilter.Filter_Average(Left_Median);                 //带权均值滤波
	//Joint Left_Particle = LeftHandFilter.Filter_Particle(Left_Median);
	pdrawlist.clear();
	brushlist.clear();
	p2draw = 2;
	pdrawlist.push_back(Right_Average);
	//pdrawlist.push_back(Right_LS);
	//pdrawlist.push_back(Right_Particle);  //左手黑点
	pdrawlist.push_back(joints[JointType_HandRight]);

	brushlist.push_back(BrushBlue);
	brushlist.push_back(BrushBlack);

	//肩膀均值滤波
	ljxCalShoulderPos(joints[JointType_ShoulderRight], Right_Median, hsRight, ljx_m_sRight);
	//ljxCalShoulderPos(joints[JointType_ShoulderRight], Right_Particle, hsRight, ljx_m_sRight);
	//ljxCalShoulderPos(joints[JointType_ShoulderLeft], Left_Average, hsLeft, ljx_m_sLeft);
	ljxCalShoulderPos(joints[JointType_ShoulderLeft], Left_LS, hsLeft, ljx_m_sLeft);

	//录制与识别
	if (ljx_m_bRecording)
	{
		if (record && recording)
		{
			ljxWriteRecord(joints[JointType_HandRight], Right_Median, hsRight);
		}
		Joint Relative;
		Relative.Position.X = Right_Average.Position.X - ljx_m_sRight.ShoulderCenter.X;
		Relative.Position.Y = Right_Average.Position.Y - ljx_m_sRight.ShoulderCenter.Y;
		Relative.Position.Z = Right_Average.Position.Z - ljx_m_sRight.ShoulderCenter.Z;
		ljxRecognize(Relative);
	}

	ljxResponse();
}

void CBodyBasics::ljxCalShoulderPos(Joint Shoulder, Joint Hand, HandState state, Side &side)
{
	side.ShoulderLast = side.Shoulder;
	side.Shoulder = Shoulder;
	side.HandLast = side.Hand;
	side.Hand = Hand;
	if ((state != HandState_NotTracked && state != HandState_Unknown))
	{
		side.StateLast = side.State;
		side.State = state;
	}
	if (Shoulder.TrackingState == TrackingState_NotTracked)
	{
		//未检测到肩膀
		return;
	}
	if (ljxJointDistance(side.Shoulder, side.ShoulderLast) > ljx_c_dDistanceThreshold)
	{
		//肩膀移动过快，说明是用户移动了
		side.Count = 1;
	}
	side.ShoulderCenter.X = (side.ShoulderCenter.X*side.Count + Shoulder.Position.X) / (side.Count + 1);
	side.ShoulderCenter.Y = (side.ShoulderCenter.Y*side.Count + Shoulder.Position.Y) / (side.Count + 1);
	side.ShoulderCenter.Z = (side.ShoulderCenter.Z*side.Count + Shoulder.Position.Z) / (side.Count + 1);
	side.Count++;
}

void CBodyBasics::ljxResponse()
{
	//左手动作 - 前进(w)
	if (ljx_m_sLeft.State != HandState_NotTracked)
	{
		//if (ljx_m_sLeft.State == HandState_Closed)
		//{
		//	INPUT moveforward;
		//	moveforward.type = INPUT_KEYBOARD;
		//	moveforward.ki.wVk = 0x57;
		//	moveforward.ki.dwFlags = 0;
		//	moveforward.ki.time = 0;
		//	moveforward.ki.dwExtraInfo = 0;
		//	if (sendinput) SendInput(1, &moveforward, sizeof(moveforward));

		//	Sleep(10);
		//	moveforward.type = INPUT_KEYBOARD;
		//	moveforward.ki.wVk = 0x57;
		//	moveforward.ki.dwFlags = KEYEVENTF_KEYUP;
		//	moveforward.ki.time = 0;
		//	moveforward.ki.dwExtraInfo = 0;
		//	if (sendinput) SendInput(1, &moveforward, sizeof(moveforward));
		//}
		if (ljx_m_sLeft.State != ljx_m_sLeft.StateLast)
		{
			if (ljx_m_sLeft.State == HandState_Closed && ljx_m_sLeft.StateLast == HandState_Open)
			{
				INPUT moveforward;
				moveforward.type = INPUT_KEYBOARD;
				moveforward.ki.wVk = 0x57;
				moveforward.ki.dwFlags = 0;
				moveforward.ki.time = 0;
				moveforward.ki.dwExtraInfo = 0;
				if (sendinput) SendInput(1, &moveforward, sizeof(moveforward));
			}
			if (ljx_m_sLeft.State == HandState_Open && ljx_m_sLeft.StateLast == HandState_Closed)
			{
				INPUT moveforward;
				moveforward.type = INPUT_KEYBOARD;
				moveforward.ki.wVk = 0x57;
				moveforward.ki.dwFlags = KEYEVENTF_KEYUP;
				moveforward.ki.time = 0;
				moveforward.ki.dwExtraInfo = 0;
				if (sendinput) SendInput(1, &moveforward, sizeof(moveforward));
			}
			if (ljx_m_sLeft.State == HandState_Lasso && ljx_m_sLeft.StateLast == HandState_Open)
			{
				INPUT mouse;
				ZeroMemory(&mouse, sizeof(mouse));
				mouse.type = INPUT_MOUSE;
				mouse.mi.dx = 0;
				mouse.mi.dy = 0;
				mouse.mi.mouseData = 0;
				mouse.mi.dwExtraInfo = GetMessageExtraInfo();
				mouse.mi.time = GetTickCount();
				mouse.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
				if (sendinput) SendInput(1, &mouse, sizeof(mouse));
			}
			if (ljx_m_sLeft.State == HandState_Open && ljx_m_sLeft.StateLast == HandState_Lasso)
			{
				INPUT mouse;
				ZeroMemory(&mouse, sizeof(mouse));
				mouse.type = INPUT_MOUSE;
				mouse.mi.dx = 0;
				mouse.mi.dy = 0;
				mouse.mi.mouseData = 0;
				mouse.mi.dwExtraInfo = GetMessageExtraInfo();
				mouse.mi.time = GetTickCount();
				mouse.mi.dwFlags = MOUSEEVENTF_LEFTUP;
				if (sendinput) SendInput(1, &mouse, sizeof(mouse));
			}
		}
	}

	Joint Relative;
	Relative.Position.X = ljx_m_sLeft.Hand.Position.X - ljx_m_sLeft.ShoulderCenter.X;
	Relative.Position.Y = ljx_m_sLeft.Hand.Position.Y - ljx_m_sLeft.ShoulderCenter.Y;
	Relative.Position.Z = ljx_m_sLeft.Hand.Position.Z - ljx_m_sLeft.ShoulderCenter.Z;
	static Filter joystick;
	Joint joy = joystick.JoyStick(Relative);

	INPUT mouse;
	double dx, dy;

	int xsize = 10000, ysize = 8000;
	dx = joy.Position.X;
	dy = joy.Position.Y;
	ZeroMemory(&mouse, sizeof(mouse));
	mouse.type = INPUT_MOUSE;
	mouse.mi.dx = (int)(dx * xsize);
	mouse.mi.dy = (int)(-dy * ysize);
	mouse.mi.dwFlags = MOUSEEVENTF_MOVE;
	if (sendinput)
	{
		SendInput(1, &mouse, sizeof(mouse));
	}

	//右手动作
	if (ljx_m_sRight.State != HandState_NotTracked)
	{
		if (ljx_m_sRight.State != ljx_m_sRight.StateLast)
		{
			if (ljx_m_sRight.State == HandState_Closed && ljx_m_sRight.StateLast == HandState_Open)
			{
				ljx_m_bRecording = true;
				if (record)
				{
					ljxStartRecording();
					recording = true;
				}
				if (!templates.empty())
				{
					ljxStartRecognize();
				}
			}
			if (ljx_m_sRight.State == HandState_Open && ljx_m_sRight.StateLast == HandState_Closed)
			{
				ljx_m_bRecording = false;
				if (record && recording)
				{
					ljxEndRecording();
					recording = false;
				}
				if (!templates.empty())
				{
					ljxEndRecognize();
				}
			}
		}
	}
}

double CBodyBasics::ljxJointDistance(Joint p1, Joint p2)
{
	double dx = p1.Position.X - p2.Position.X;
	double dy = p1.Position.Y - p2.Position.Y;
	return sqrt(dx*dx + dy*dy);
}

void CBodyBasics::ljxStartRecording()
{
	time_t timet;
	struct tm *timeinfo;
	time(&timet);
	timeinfo = localtime(&timet);
	memset(name, 0, sizeof(name));
	sprintf(name, "%d%d%d_%d%d%d.txt", timeinfo->tm_year + 1900,
					timeinfo->tm_mon + 1,
					timeinfo->tm_mday,
					timeinfo->tm_hour,
					timeinfo->tm_min,
					timeinfo->tm_sec);

	ljx_m_pFile = fopen(name, "w");
}

void CBodyBasics::ljxEndRecording()
{
	fprintf(ljx_m_pFile, "-999\n");
	fclose(ljx_m_pFile);
	char cmd[1000];
	memset(cmd, 0, sizeof(cmd));
	//sprintf(cmd, "F:\\Code\\Kinect\\GestureTestNew\\GestureTestNew\\GestureTestNew\\Debug\\GestureTestNew.exe %s", name);
	//system(cmd);
}

void CBodyBasics::ljxWriteRecord(Joint RightShoulder, Joint RightHand, HandState hsRightHand)
{
	if (!ljx_m_pFile)
	{
		return;
	}
	fprintf(ljx_m_pFile, "%d %0.8lf %0.8lf %0.8lf ",
					RightShoulder.TrackingState,
					RightShoulder.Position.X,
					RightShoulder.Position.Y,
					RightShoulder.Position.Z);
	fprintf(ljx_m_pFile, "%d %0.8lf %0.8lf %0.8lf %d\n",
					RightHand.TrackingState,
					RightHand.Position.X,
					RightHand.Position.Y,
					RightHand.Position.Z,
					hsRightHand);
}

void CBodyBasics::ljxStartRecognize()
{
	points.clear();
	ljx_m_nCount = 0;
}

void CBodyBasics::ljxEndRecognize()
{
	if (templates.empty())
	{
		return;
	}
	if (points.empty())
	{
		return;
	}
	points = normalize(points);
	int j = 0;

	pair<int, double> result = recognize(points, templates);

	if (result.second <= 0.4)
	{
		switch (actionlist[result.first].type)
		{
			case Action::Command:
				MessageBoxA(NULL, actionlist[result.first].action.c_str(), "", MB_OK);
				system(actionlist[result.first].action.c_str());
				break;
			case Action::Mouse:
				INPUT mouse;
				mouse.type = INPUT_MOUSE;
				mouse.mi.dx = 0;
				mouse.mi.dy = 0;
				if (actionlist[result.first].mouse == MOUSEEVENTF_WHEEL)
				{
					mouse.mi.mouseData = actionlist[result.first].wheelamount;
				}
				else
				{
					mouse.mi.mouseData = 0;
				}
				mouse.mi.dwExtraInfo = GetMessageExtraInfo();
				mouse.mi.time = GetTickCount();
				mouse.mi.dwFlags = actionlist[result.first].mouse;
				SendInput(1, &mouse, sizeof(mouse));
				break;
			case Action::Key_VK:
				INPUT vk;
				vk.type = INPUT_KEYBOARD;
				vk.ki.wVk = actionlist[result.first].vk;
				vk.ki.dwExtraInfo = GetMessageExtraInfo();
				vk.ki.time = GetTickCount();
				vk.ki.dwFlags = 0;
				SendInput(1, &vk, sizeof(vk));
				Sleep(30);
				vk.ki.dwFlags = KEYEVENTF_KEYUP;
				vk.ki.time = GetTickCount();
				SendInput(1, &vk, sizeof(vk));
				break;
			default:
				break;
		}
	}
	points.clear();
}

void CBodyBasics::ljxRecognize(Joint joint)
{
	if (templates.empty())
	{
		return;
	}
	double dx, dy;
	dx = joint.Position.X;
	dy = joint.Position.Y;
	points.push_back(VEC(dx, -dy));
	ljx_m_nCount++;
}

void CBodyBasics::ljxShowJoint()
{
	if (drawnewtemp || showtemp)
	{
		return;
	}
	static vector<D2D1_POINT_2F> drawlist;
	RECT rc;
	GetWindowRect(GetDlgItem(m_hWnd, IDC_TEMPVIEW), &rc);

	int width = rc.right - rc.left;
	int height = rc.bottom - rc.top;

	m_pTempRenderTarget->BeginDraw();
	m_pTempRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::White));

	static int i = 1;
	D2D1_POINT_2F NewPos;
	D2D1_POINT_2F MousePos;
	D2D1_POINT_2F LeftHand;

	Joint Relative;
	Relative.Position.X = ljx_m_sLeft.Hand.Position.X - ljx_m_sLeft.HandLast.Position.X;
	Relative.Position.Y = ljx_m_sLeft.Hand.Position.Y - ljx_m_sLeft.HandLast.Position.Y;
	Relative.Position.Z = ljx_m_sLeft.Hand.Position.Z - ljx_m_sLeft.HandLast.Position.Z;
	static Filter joystick;
	Joint joy = joystick.JoyStick(Relative);

	double dx, dy;
	dx = joy.Position.X;
	dy = joy.Position.Y;
	dx = (dx * width / 2 + width / 2);
	dy = -dy * height / 2 + height / 2;
	MousePos.x = dx;
	MousePos.y = dy;

	D2D1_ELLIPSE LeftMouse = D2D1::Ellipse(MousePos, 3.0, 3.0);
	//m_pTempRenderTarget->FillEllipse(LeftMouse, BrushBlue);

	LeftHand.x = ljx_m_sLeft.Hand.Position.X * width / 2 + width / 2;
	LeftHand.y = -ljx_m_sLeft.Hand.Position.Y * height / 2 + height / 2;
	D2D1_ELLIPSE Left = D2D1::Ellipse(LeftHand, 3.0, 3.0);
	m_pTempRenderTarget->FillEllipse(Left, BrushBlack);

	for (int i = 0; i < p2draw; i++)
	{
		D2D1_POINT_2F pos;
		pos.x = pdrawlist[i].Position.X * width / 2 + width / 2;
		pos.y = -pdrawlist[i].Position.Y * height / 2 + height / 2;
		D2D1_ELLIPSE ellipse = D2D1::Ellipse(pos, 3.0, 3.0);
		m_pTempRenderTarget->FillEllipse(ellipse, brushlist[i]);
	}

	NewPos.x = ljx_m_sRight.Hand.Position.X * width / 2 + width / 2;
	NewPos.y = -ljx_m_sRight.Hand.Position.Y * height / 2 + height / 2;
	if (ljx_m_bRecording)
	{
		D2D1_ELLIPSE Pos = D2D1::Ellipse(NewPos, 3.0, 3.0);;
		if (ljx_m_sRight.State==HandState_Open) m_pTempRenderTarget->FillEllipse(Pos, BrushRed);
		else if (ljx_m_sRight.State == HandState_Closed) m_pTempRenderTarget->FillEllipse(Pos, BrushGreen);
		else m_pTempRenderTarget->FillEllipse(Pos, BrushBlack);
		drawlist.push_back(NewPos);
		i++;
		for (vector<D2D1_POINT_2F>::iterator i = drawlist.begin();
				 i != drawlist.end() && i + 1 != drawlist.end();
				 i++)
		{
			m_pTempRenderTarget->DrawLine(*i, *(i + 1), BrushBlack);
			
		}
		for (vector<D2D1_POINT_2F>::iterator i = drawlist.begin();
				 i != drawlist.end(); i++)
		{
			D2D1_ELLIPSE Pos = D2D1::Ellipse(*i, 1.0, 1.0);;
			m_pTempRenderTarget->FillEllipse(Pos, BrushRed);
		}
	}
	else
	{
		if (ljx_m_sRight.State != HandState_NotTracked)
		{
			D2D1_ELLIPSE Pos = D2D1::Ellipse(NewPos, 3.0, 3.0);;
			m_pTempRenderTarget->FillEllipse(Pos, BrushRed);
		}
		drawlist.clear();
	}
	m_pTempRenderTarget->EndDraw();
}

void CBodyBasics::ljxStartDrawTemp()
{
	points.clear();
	ljx_m_vNewTemp.clear();
}

void CBodyBasics::ljxDrawTemplate(int x, int y, int width, int height)
{
	points.push_back(VEC(x, y));

	m_pTempRenderTarget->BeginDraw();
	m_pTempRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::White));
	static int i = 1;
	D2D1_POINT_2F NewPos;
	NewPos.x = x;
	NewPos.y = y;
	ljx_m_vNewTemp.push_back(NewPos);
	D2D1_ELLIPSE Pos = D2D1::Ellipse(NewPos, 3.0, 3.0);;
	m_pTempRenderTarget->FillEllipse(Pos, BrushRed);
	for (vector<D2D1_POINT_2F>::iterator i = ljx_m_vNewTemp.begin();
			 i != ljx_m_vNewTemp.end(); i++)
	{
		D2D1_ELLIPSE Pos = D2D1::Ellipse(*i, 1.0, 1.0);;
		m_pTempRenderTarget->FillEllipse(Pos, BrushRed);
	}
	for (vector<D2D1_POINT_2F>::iterator i = ljx_m_vNewTemp.begin();
			 i != ljx_m_vNewTemp.end() && i + 1 != ljx_m_vNewTemp.end();
			 i++)
	{
		m_pTempRenderTarget->DrawLine(*i, *(i + 1), BrushBlack);

	}
	m_pTempRenderTarget->EndDraw();
}

void CBodyBasics::ljxEndDrawTemp()
{
	ljx_m_vNewTemp.clear();
	points = normalize(points);

	IFileDialog *pfd;
	HRESULT hr = CoCreateInstance(CLSID_FileSaveDialog, NULL, CLSCTX_INPROC_SERVER,
																IID_PPV_ARGS(&pfd));
	DWORD dwFlags;
	hr = pfd->GetOptions(&dwFlags);
	hr = pfd->SetOptions(dwFlags | FOS_FORCEFILESYSTEM);
	COMDLG_FILTERSPEC filter[] = {
		{ L"All File", L"*.*" }
	};
	hr = pfd->SetFileTypes(1, filter);
	hr = pfd->SetFileTypeIndex(1);
	hr = pfd->Show(m_hWnd);
	if (hr == S_OK)
	{
		IShellItem *result;
		hr = pfd->GetResult(&result);
		LPWSTR fileName;
		LPWSTR filePath;
		hr = result->GetDisplayName(SIGDN_NORMALDISPLAY, &fileName);
		hr = result->GetDisplayName(SIGDN_DESKTOPABSOLUTEPARSING, &filePath);
		ofstream newtemp(filePath);
		for (Points::iterator i = points.begin();
				 i != points.end(); i++)
		{
			newtemp << i->x << " " << i->y << endl;
		}
		newtemp.close();
	}
	points.clear();
}

void CBodyBasics::ljxShowTemplate()
{
	if (!showtemp)
	{
		return;
	}

	if (ljx_m_vTempShowing.empty())
	{
		MessageBoxA(NULL, "temp empty!", "empty!", MB_OK);
		return;
	}

	RECT rc;
	GetWindowRect(GetDlgItem(m_hWnd, IDC_TEMPVIEW), &rc);

	int width = rc.right - rc.left;
	int height = rc.bottom - rc.top;

	m_pTempRenderTarget->BeginDraw();
	m_pTempRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::White));

	for (vector<VEC>::iterator i = ljx_m_vTempShowing.begin();
			 i != ljx_m_vTempShowing.end();
			 i++)
	{
		D2D1_POINT_2F NewPos;
		NewPos.x = ((i)->x + 250.0) / 500.0*width;
		NewPos.y = ((i)->y + 250.0) / 500.0*width;
		D2D1_ELLIPSE Pos = D2D1::Ellipse(NewPos, 2.0, 2.0);;
		D2D1_ELLIPSE BeginPos = D2D1::Ellipse(NewPos, 4.0, 4.0);;
		if (i == ljx_m_vTempShowing.begin())
		{
			m_pTempRenderTarget->FillEllipse(BeginPos, BrushRed);
		}
		else
		{
			m_pTempRenderTarget->FillEllipse(Pos, BrushRed);
		}
	}

	for (vector<VEC>::iterator i = ljx_m_vTempShowing.begin();
			 i != ljx_m_vTempShowing.end() && i+1 != ljx_m_vTempShowing.end();
			 i++)
	{
		D2D1_POINT_2F NewPos;
		NewPos.x = ((i + 1)->x + 250.0) / 500.0*width;
		NewPos.y = ((i + 1)->y + 250.0) / 500.0*width;
		D2D1_POINT_2F Pos;
		Pos.x = ((i)->x + 250.0) / 500.0*width;
		Pos.y = ((i)->y + 250.0) / 500.0*width;
		m_pTempRenderTarget->DrawLine(Pos, NewPos, BrushBlack);
	}

	m_pTempRenderTarget->EndDraw();

}

void CBodyBasics::ljxInitialize()
{
	RECT rc;
	GetWindowRect(GetDlgItem(m_hWnd, IDC_TEMPVIEW), &rc);

	int width = rc.right - rc.left;
	int height = rc.bottom - rc.top;

	D2D1_SIZE_U size = D2D1::SizeU(width, height);
	D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
	rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
	rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

	HRESULT hr = m_pTempFactory->CreateHwndRenderTarget(
		rtProps,
		D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_TEMPVIEW), size),
		&m_pTempRenderTarget
		);

	m_pTempRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Black), &BrushBlack);
	m_pTempRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red), &BrushRed);
	m_pTempRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue), &BrushBlue);
	m_pTempRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green), &BrushGreen);
}