/*
 *  cam_FrameMon.cpp
 *  PHD Guiding
 *
 *  Created by Leo Shatz.
 *  Copyright (c) 2024-2025 Leo Shatz
 *  Copyright (c) 2018-2025 openphdguiding.org
 *  All rights reserved.
 *
 *  This source code is distributed under the following "BSD" license
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *    Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *    Neither the name of Craig Stark, Stark Labs nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

# include "phd.h"
# include "camera.h"

#if defined(FRAME_MONITOR_CAMERA)

# include <opencv2/opencv.hpp>
# include "cam_FrameMon.h"
# include <wx/socket.h>

# define FRAME_MONITOR_TIMEOUT_MS 10000
# define FRAME_IMAGE_BUFFER_SIZE (2048 * 2048 * 2)
# define IMAGE_LINK_ID ":if:"

class ImageFrameServer;

// Image frame header (should be multiple of 4 bytes)
struct imageFrameHeader
{
# define IFLINK_MAGIC 0x46C9A3D0
# define IFLINK_VERSION 2
    volatile uint32_t magic;
    volatile uint16_t version;
    volatile uint16_t hdrLength;
    volatile uint32_t dataLength;
    uint16_t width;
    uint16_t height;
    double pixelSize;

    unsigned short detected;
    unsigned short features;
    unsigned short radius;
    unsigned short minRadius;
    unsigned short maxRadius;
    unsigned short peak;
    float centerX;
    float centerY;
    float dispersion;
    float quality;
    float sharpness;
    float snr;
    float mass;
    float detectionTime;
};

class ImageFrameClientHandler : public wxEvtHandler
{
public:
    ImageFrameClientHandler(wxSocketBase *sock, ImageFrameServer *server);
    ~ImageFrameClientHandler();

    void Destroy();
    void ProcessImage();
    void ReadFrame();
    void OnSocketEvent(wxSocketEvent& event);
    ImageFrameServer *imgServer;
    wxSocketBase *imgSock;

private:
    imageFrameHeader hdr;

    char *imgBuffer;
    uint32_t bytesReceived;
    bool headerReceived;

    void SetSocketOptions();
    void ResetState();

    wxDECLARE_EVENT_TABLE();
};

class ImageFrameServer : public wxEvtHandler
{
public:
    ImageFrameServer(unsigned short port);
    ~ImageFrameServer();

    bool StartServer();
    void StopServer();
    void AddClient(ImageFrameClientHandler *cli, wxSocketBase *sock);
    void RemoveClient(ImageFrameClientHandler *cli);
    void DestroyClient();
    void StopClient(bool stop);
    bool WaitClientStopped(int msecTimeout);
    bool IsConnected();
    bool IsClientStopping();
    bool IsServerStopping() { return stop_flag; }
    void AddImageFrame(char *buf, imageFrameHeader& hdr);
    bool GetImageFrame(cv::Mat& frame, bool flush, frameDesc& desc);

private:
    friend class ImageServerThread;
    void OnServerEvent(wxSocketEvent& event);

    wxCriticalSection clientsLock;
    ImageFrameClientHandler *client;
    wxSocketBase *clientSock;
    wxMutex stopMutex;
    wxCondition stopCond;
    bool clientStopping;

    wxThread *thread;
    wxSocketServer *serverSocket;
    unsigned short imgPort;
    bool stop_flag;
    bool connected_flag;

    wxCriticalSection imgLock;
    cv::Mat imgMat;
    frameDesc imgDesc;

    wxDECLARE_EVENT_TABLE();
};

class ImageServerThread : public wxThread
{
public:
    ImageServerThread(ImageFrameServer *socket);
    virtual ~ImageServerThread();

protected:
    virtual ExitCode Entry();

private:
    ImageFrameServer *imgServer;
};

wxBEGIN_EVENT_TABLE(ImageFrameClientHandler, wxEvtHandler) EVT_SOCKET(wxID_ANY, ImageFrameClientHandler::OnSocketEvent)
    wxEND_EVENT_TABLE()

ImageFrameClientHandler::ImageFrameClientHandler(wxSocketBase *sock, ImageFrameServer *server)
    : imgSock(sock), imgServer(server)
{
    imgBuffer = new char[FRAME_IMAGE_BUFFER_SIZE];
    assert(imgBuffer);
    imgSock->SetEventHandler(*this, wxID_ANY);
    imgSock->SetNotify(wxSOCKET_LOST_FLAG);
    imgSock->Notify(true);
    server->AddClient(this, sock);
    SetSocketOptions();
    ResetState();
}

ImageFrameClientHandler::~ImageFrameClientHandler()
{
    delete[] imgBuffer;
}

void ImageFrameClientHandler::Destroy()
{
    Disconnect(wxEVT_SOCKET, wxSocketEventHandler(ImageFrameClientHandler::OnSocketEvent), NULL, this);
    if (imgSock)
    {
        imgSock->Destroy();
        imgSock = nullptr;
    }
    imgServer->RemoveClient(this);
    CallAfter([this]() { delete this; });
}

void ImageFrameClientHandler::ResetState()
{
    hdr.dataLength = 0;
    bytesReceived = 0;
    headerReceived = false;
}

void ImageFrameClientHandler::ProcessImage()
{
    PHD_Point loc = PHD_Point(hdr.centerX, hdr.centerY);
    if (!hdr.detected)
    {
        loc.Invalidate();
    }
    imgServer->AddImageFrame(imgBuffer, hdr);
    wxDateTime now = wxDateTime::UNow();
    wxString cameraName = pCamera ? pCamera->GetStrProperty("name") : wxEmptyString;
    wxString ts = IMAGE_LINK_ID + cameraName + _(": ");
    ts += now.Format(wxT("%H:%M:%S")) + wxString::Format(".%02d", now.GetMillisecond() / 10);
    if (pCamera)
        pCamera->SetProperty("path_broadcast", ts);
}

void ImageFrameClientHandler::SetSocketOptions()
{
    int size = 65536, opt = 1;
    imgSock->SetOption(SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
    imgSock->SetOption(IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    imgSock->SetTimeout(1);
}

void ImageFrameClientHandler::ReadFrame()
{
    char *header = reinterpret_cast<char *>(&hdr);
    int headerSize = sizeof(hdr);
    char temp[1024];
    do
    {
        if (imgServer->IsServerStopping() || imgServer->IsClientStopping())
            break;

        if (!headerReceived)
        {
            imgSock->Read(header + bytesReceived, headerSize - bytesReceived);
            bytesReceived += imgSock->LastReadCount();

            if (bytesReceived < sizeof(hdr))
                continue;
            if (hdr.hdrLength < 1024)
            {
                headerSize = hdr.hdrLength;
                header = temp;
            }
            if (bytesReceived < hdr.hdrLength)
                continue;

            if ((hdr.magic != IFLINK_MAGIC) || (hdr.version < IFLINK_VERSION) ||
                (hdr.hdrLength < sizeof(hdr)) || (hdr.dataLength > FRAME_IMAGE_BUFFER_SIZE))
            {
                Debug.Write(wxString::Format(FRAME_MONITOR_CAMERA ": invalid frame: magic=%x, v=%x, l=%d\n", hdr.magic, hdr.version, hdr.dataLength));
                imgServer->StopClient(true);
                break;
            }

            headerReceived = true;
            bytesReceived = 0;
            continue;
        }

        uint32_t limit = wxMin(FRAME_IMAGE_BUFFER_SIZE, hdr.dataLength);
        if (bytesReceived < limit)
        {
            imgSock->Read(imgBuffer + bytesReceived, limit - bytesReceived);
            bytesReceived += imgSock->LastReadCount();
        }

        if (bytesReceived == hdr.dataLength)
        {
            ProcessImage();
            ResetState();
            break;
        }
    } while (imgSock->WaitForRead(0, 100) && imgSock->IsConnected() && !imgSock->IsClosed());
}

void ImageFrameClientHandler::OnSocketEvent(wxSocketEvent& event)
{
    if (event.GetSocketEvent() == wxSOCKET_LOST)
    {
        if (event.GetSocket() == imgSock)
            imgServer->StopClient(true);
    }
}

wxBEGIN_EVENT_TABLE(ImageFrameServer, wxEvtHandler) EVT_SOCKET(FRAME_MONITOR_ID, ImageFrameServer::OnServerEvent)
    wxEND_EVENT_TABLE()

ImageFrameServer::ImageFrameServer(unsigned short port)
    : imgPort(port), serverSocket(nullptr), thread(nullptr), stop_flag(false), connected_flag(false), client(nullptr),
      clientSock(nullptr), clientStopping(false), stopCond(stopMutex)
{
}

ImageFrameServer::~ImageFrameServer()
{
    StopServer();
}

bool ImageFrameServer::StartServer()
{
    if (!thread)
    {
        thread = new ImageServerThread(this);
        if (thread->Create() != wxTHREAD_NO_ERROR)
        {
            Debug.Write(FRAME_MONITOR_CAMERA ": failed to create thread!");
            delete thread;
            thread = nullptr;
            return false;
        }
        else
        {
            thread->Run();
        }
    }
    return true;
}

void ImageFrameServer::DestroyClient()
{
    wxCriticalSectionLocker locker(clientsLock);
    if (client)
        client->Destroy();
}

void ImageFrameServer::StopServer()
{
    stop_flag = true;

    if (thread)
    {
        thread->Wait();
        delete thread;
        thread = nullptr;
    }
    if (serverSocket)
    {
        serverSocket->Destroy();
        serverSocket = nullptr;
    }

    DestroyClient();
}

void ImageFrameServer::StopClient(bool stopping)
{
    wxMutexLocker locker(stopMutex);
    clientStopping = stopping;
    if (stopping)
        connected_flag = false;
    if (!stopping)
        stopCond.Signal();
}

bool ImageFrameServer::IsClientStopping()
{
    wxMutexLocker locker(stopMutex);
    return clientStopping;
}

bool ImageFrameServer::IsConnected()
{
    return connected_flag;
}

bool ImageFrameServer::WaitClientStopped(int msecTimeout)
{
    wxMutexLocker locker(stopMutex);
    if (!clientStopping)
        return false;
    stopCond.WaitTimeout(msecTimeout);
    return clientStopping;
}

void ImageFrameServer::AddImageFrame(char *buf, imageFrameHeader& hdr)
{
    try
    {
        cv::Mat tmp(hdr.height, hdr.width, CV_16UC(1), buf);
        wxCriticalSectionLocker locker(imgLock);
        imgMat = tmp.clone();
        imgDesc.pos = PHD_Point(hdr.centerX, hdr.centerY);
        imgDesc.pixelSize = hdr.pixelSize;
        imgDesc.features = hdr.features;
        imgDesc.dispersion = hdr.dispersion;
        imgDesc.quality = hdr.quality;
        imgDesc.sharpness = hdr.sharpness;
        imgDesc.snr = hdr.snr;
        imgDesc.mass = hdr.mass;
        imgDesc.peak = hdr.peak;
        imgDesc.time = hdr.detectionTime;
        imgDesc.minRadius = hdr.minRadius;
        imgDesc.maxRadius = hdr.maxRadius;
        if (hdr.detected)
        {

            imgDesc.radius = hdr.radius;
        }
        else
        {
            imgDesc.pos.Invalidate();
        }
    }
    catch (const cv::Exception& e)
    {
        Debug.Write(wxString::Format(FRAME_MONITOR_CAMERA ": exception: %s\n", e.what()));
    }
}

bool ImageFrameServer::GetImageFrame(cv::Mat& frame, bool flush, frameDesc& desc)
{
    wxCriticalSectionLocker locker(imgLock);
    try
    {
        if (imgMat.empty())
        {
            frame = cv::Mat();
            desc = frameDesc();
            return false;
        }
        frame = imgMat.clone();
        desc = imgDesc;
        if (flush)
            imgMat = cv::Mat();
        return true;
    }
    catch (const cv::Exception& e)
    {
        Debug.Write(wxString::Format(FRAME_MONITOR_CAMERA ": exception: %s\n", e.what()));
        frame = cv::Mat();
        desc = frameDesc();
        return false;
    }
}

void ImageFrameServer::AddClient(ImageFrameClientHandler *cli, wxSocketBase *sock)
{
    wxCriticalSectionLocker locker(clientsLock);
    client = cli;
    clientSock = sock;
    connected_flag = true;
}

void ImageFrameServer::RemoveClient(ImageFrameClientHandler *cli)
{
    wxCriticalSectionLocker locker(clientsLock);
    client = nullptr;
    clientSock = nullptr;
    connected_flag = false;
}

void ImageFrameServer::OnServerEvent(wxSocketEvent& event)
{
    if (event.GetSocketEvent() != wxSOCKET_CONNECTION)
        return;

    wxSocketBase *socket = serverSocket->Accept(false);
    if (socket == nullptr)
        return;

    if (client)
    {
        if (!IsClientStopping())
        {
            socket->Destroy();
            return;
        }

        StopClient(true);
        while (!stop_flag && WaitClientStopped(100))
            ;
    }

    if (stop_flag)
    {
        socket->Destroy();
        return;
    }
    new ImageFrameClientHandler(socket, this);
}

ImageServerThread::ImageServerThread(ImageFrameServer *server) : wxThread(wxTHREAD_JOINABLE), imgServer(server) { }

ImageServerThread::~ImageServerThread() { }

wxThread::ExitCode ImageServerThread::Entry()
{
    wxIPV4address addr;
    addr.Hostname("127.0.0.1");
    addr.Service(imgServer->imgPort);

    wxSocketServer *server = new wxSocketServer(addr, wxSOCKET_REUSEADDR);

    if (!server->IsOk())
    {
        delete server;
        return (wxThread::ExitCode) 1;
    }

    imgServer->serverSocket = server;
    server->SetEventHandler(*imgServer, FRAME_MONITOR_ID);
    server->SetNotify(wxSOCKET_CONNECTION_FLAG);
    server->Notify(true);

    while (!imgServer->stop_flag && !TestDestroy())
    {
        bool doWait = true;
        {
            wxSocketBase *sock = imgServer->clientSock;
            if (sock && sock->IsConnected() && !imgServer->IsClientStopping() && imgServer->client)
            {
                imgServer->connected_flag = true;
                if (sock->WaitForRead(0, 100))
                    imgServer->client->ReadFrame();
                doWait = false;
            }
            else
                imgServer->connected_flag = false;
        }
        if (imgServer->IsClientStopping())
        {
            imgServer->DestroyClient();
            imgServer->StopClient(false);
        }
        if (doWait)
            wxMilliSleep(100);
    }

    return (wxThread::ExitCode) 0;
}

// ======================================================================

wxString GetFrameMonitorLabel()
{
    wxString framePath = pCamera ? pCamera->GetStrProperty("path") : wxEmptyString;
    if (framePath.StartsWith(IMAGE_LINK_ID))
    {
        return framePath.Mid(strlen(IMAGE_LINK_ID));
    }
    else
    {
        wxFileName fileName(framePath);
        return wxString::Format("%s", fileName.GetFullName());
    }
}

// ======================================================================

CameraFrameMonitor::CameraFrameMonitor() : m_serverCond(m_serverMutex), m_useCount(0), m_sync(m_lock)
{
    Connected = false;
    Name = FRAME_MONITOR_CAMERA;
    m_lastKnownGoodFilename = wxEmptyString;
    m_cameraConnectAlertMsg = _("Lost camera feed, waiting for reconnection ...");
    FrameSize = wxSize(640, 480);
    m_frameSize = FrameSize;
    m_hasGuideOutput = true;
    m_imageServer = nullptr;
    m_lastKnownImage = cv::Mat::zeros(m_frameSize.y, m_frameSize.x, CV_16UC1);

    m_imgPort = 0;
    m_ready = false;
    m_path = wxEmptyString;
    m_physName = wxEmptyString;
}

CameraFrameMonitor::~CameraFrameMonitor(void)
{
    ImageFrameServer *server = nullptr;

    {
        wxMutexLocker locker(m_serverMutex);
        while (m_imageServer && m_useCount > 0)
            m_serverCond.WaitTimeout(100);
        server = m_imageServer;
        m_imageServer = nullptr;
    }

    if (server)
    {
        server->StopServer();
        delete server;
    }
}

// Ask for access to image server and increment reference counter
bool CameraFrameMonitor::GetServer()
{
    wxMutexLocker locker(m_serverMutex);
    if (m_imageServer == nullptr)
        return false;
    m_useCount++;
    return true;
}

// Release reference
void CameraFrameMonitor::PutServer()
{
    // Check to see if connection exists and increment referece counter
    wxMutexLocker locker(m_serverMutex);
    if (m_useCount > 0)
        m_useCount--;
    m_serverCond.Signal();
}

wxByte CameraFrameMonitor::BitsPerPixel()
{
    return 16;
}

void CameraFrameMonitor::StartImageServer()
{
    uint16_t imgPort = m_imgPort;
    if (imgPort == 0)
        return;

    {
        wxMutexLocker locker(m_serverMutex);
        if (m_imageServer)
            return;
    }

    ImageFrameServer *server = new ImageFrameServer(imgPort);
    if (!server->StartServer())
    {
        pFrame->Alert(_(FRAME_MONITOR_CAMERA ": failed to establish image link!"));
        Debug.Write(wxString::Format(FRAME_MONITOR_CAMERA ": failed to establish image link on port %d\n", imgPort));
        delete server;
    }
    else
    {
        wxMutexLocker locker(m_serverMutex);
        m_imageServer = server;
    }
}

void CameraFrameMonitor::InitCapture()
{
    if (Connected)
        StartImageServer();
}

bool CameraFrameMonitor::Connect(const wxString& camId)
{
    bool bError = false;
    SetProperty("path", wxEmptyString);
    Connected = true;
    StartImageServer();
    return bError;
}

bool CameraFrameMonitor::Disconnect()
{
    ImageFrameServer *server = nullptr;
    {
        wxMutexLocker locker(m_serverMutex);
        while (m_imageServer && m_useCount > 0)
            m_serverCond.WaitTimeout(100);
        server = m_imageServer;
        m_imageServer = nullptr;
    }

    if (server)
    {
        server->StopServer();
        delete server;
    }

    pFrame->ClearAlert(m_cameraConnectAlertMsg);

    Connected = false;
    return false;
}

bool CameraFrameMonitor::Capture(int duration, usImage& img, int options, const wxRect& subframe)
{
    bool bError = false;
    frameDesc imgDesc;
    wxStopWatch swatch;

    try
    {
        cv::Mat image;
        bool paused;
        int timeout = pFrame->IsCaptureActive(paused) ? FRAME_MONITOR_TIMEOUT_MS + duration : 0;
        wxString filename = GetStrProperty("path", timeout);
        Debug.Write(wxString::Format(FRAME_MONITOR_CAMERA ": latency %d ms (to:%d)\n", swatch.Time(), timeout));

        if (filename == wxString("NUL"))
            filename = m_lastKnownGoodFilename;

        bool connected = true;
        if (filename.StartsWith(IMAGE_LINK_ID))
        {
            if (GetServer())
            {
                connected = m_imageServer->IsConnected();
                double oldPixelSize = pCamera ? pCamera->GetCameraPixelSize() : 0;
                if (m_imageServer->GetImageFrame(image, !paused, imgDesc))
                {
                    if (pCamera && m_imgDesc.pixelSize != oldPixelSize && m_imgDesc.pixelSize != UnknownPixelSize)
                        pCamera->SetCameraPixelSize(m_imgDesc.pixelSize);
                }
                {
                    wxMutexLocker lck(m_lock);
                    m_imgDesc = imgDesc;
                }
                PutServer();
            }
            else
            {
                connected = false;
                image = m_lastKnownImage;
            }
        }
        if (image.empty())
        {
            bool paused;
            if (timeout && pFrame->IsCaptureActive(paused))
                pFrame->Alert(m_cameraConnectAlertMsg);

            image = m_lastKnownImage;
        }
        else
        {
            pFrame->ClearAlert(m_cameraConnectAlertMsg);
            m_lastKnownGoodFilename = filename;
            m_lastKnownImage = image.clone();
        }

        if (img.Init(image.cols, image.rows))
        {
            pFrame->Alert(_("Memory allocation error"));
            return true;
        }

        FrameSize.x = image.size().width;
        FrameSize.y = image.size().height;

        cv::Mat *disk_image = &image;
        cv::Mat grayscaleImage;
        cv::Mat grayscale16;

        if (disk_image->channels() != 1)
        {
            cvtColor(image, grayscaleImage, cv::COLOR_BGR2GRAY);
            disk_image = &grayscaleImage;
        }
        if (disk_image->depth() != CV_16U)
        {
            disk_image->convertTo(grayscale16, CV_16UC1, 65535.0 / 255.0);
            disk_image = &grayscale16;
        }

        m_frameSize = FrameSize;
        int dataSize = image.cols * image.rows * 2;
        memcpy(img.ImageData, disk_image->data, dataSize);
    }
    catch (const cv::Exception& ex)
    {
        Debug.Write(wxString::Format(FRAME_MONITOR_CAMERA ": OpenCV exception %s\n", ex.what()));
        bError = true;
    }

    return bError;
}

bool CameraFrameMonitor::ST4PulseGuideScope(int direction, int duration)
{
    char dir;
    switch (direction)
    {
    case WEST:
        dir = 'W';
        break;
    case NORTH:
        dir = 'N';
        break;
    case SOUTH:
        dir = 'S';
        break;
    case EAST:
        dir = 'E';
        break;
    default:
        return true; // bad direction passed in
    }

    EvtServer.NotifySt4Step(dir, duration);

    if (duration > 10)
        if (WorkerThread::MilliSleep(duration - 10))
            return true;

    return false;
}

bool CameraFrameMonitor::GetCaptureDescriptor(void* ptr)
{
    frameDesc *desc = static_cast<frameDesc *>(ptr);
    wxMutexLocker lck(m_lock);
    *desc = m_imgDesc;
    return false; // No error
}

void CameraFrameMonitor::SetProperty(const wxString prop, wxString value)
{
    if (prop == "path_broadcast")
    {
        wxMutexLocker lck(m_lock);
        m_path = value;
        m_ready = true;
        m_sync.Broadcast();
    }
    else if (prop == "path")
    {
        wxMutexLocker lck(m_lock);
        m_path = value;
    }
    else if (prop == "name")
    {
        m_physName = value;
    }
}

void CameraFrameMonitor::SetProperty(const wxString prop, int value)
{
    if (prop == "port")
    {
        m_imgPort = value;
        if (Connected)
        {
            InitCapture();
            pFrame->ClearAlert();
        }
    }
}

wxString CameraFrameMonitor::GetStrProperty(const wxString prop, int timeout)
{
    if (prop == "name")
        return m_physName;
    if (prop == "path")
    {
        wxStopWatch swatch;
        bool paused;
        const int fragTimeout = 100;
        if (timeout)
            EvtServer.NotifyStartCapture();
        wxMutexLocker lck(m_lock);
        while (!m_ready && timeout > 0 && pFrame->IsCaptureActive(paused))
        {
            if (m_sync.WaitTimeout(fragTimeout) == wxCOND_TIMEOUT)
            {
                timeout -= fragTimeout;
                if (swatch.Time() > wxMax(1000, pFrame->GetGuidingPeriod()))
                {
                    swatch.Start();
                    EvtServer.NotifyStartCapture();
                }
            }
        }
        m_ready = false;
        return m_path;
    }
    return wxEmptyString;
}

#else

wxString GetFrameMonitorLabel()
{
    return wxString();
}

#endif // FRAME_MONITOR_CAMERA
