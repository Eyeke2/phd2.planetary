/*
 *  cam_FrameMon.cpp
 *  PHD Guiding
 *
 *  Created by Leo Shatz.
 *  Copyright (c) 2024-2026 Leo Shatz
 *  Copyright (c) 2018-2026 openphdguiding.org
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
# include "frame_export.h"
# include "shm_frame_layout.h"

#if defined(FRAME_MONITOR_CAMERA)

# include <atomic>
# include <cmath>
# include <opencv2/opencv.hpp>
# include "cam_FrameMon.h"
# include <wx/socket.h>

# define FRAME_MONITOR_TIMEOUT_MS 10000
# define FRAME_IMAGE_BUFFER_SIZE (2048 * 2048 * 2)
# define IMAGE_LINK_ID ":if:"

// Resilience tunables for partial receives and stale single-client sessions.
# define FRAME_MONITOR_HEADER_TIMEOUT_MS 5000
# define FRAME_MONITOR_FRAME_TIMEOUT_MS  10000
# define FRAME_MONITOR_IDLE_EVICTION_MS  3000

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

    // Milliseconds since the last byte received, used for idle eviction.
    long long IdleMs() const;

    ImageFrameServer *imgServer;
    wxSocketBase *imgSock;

private:
    imageFrameHeader hdr;

    char *imgBuffer;
    uint32_t bytesReceived;
    bool headerReceived;

    // Last received-byte timestamp; read by the main thread, written by worker.
    std::atomic<long long> m_lastByteAtMs;

    // Start timestamp for the current partial header/frame body.
    long long m_partialStartMs;

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
    : imgSock(sock), imgServer(server),
      m_lastByteAtMs(::wxGetUTCTimeMillis().GetValue()),
      m_partialStartMs(0)
{
    imgBuffer = new char[FRAME_IMAGE_BUFFER_SIZE];
    assert(imgBuffer);
    SetSocketOptions();
    ResetState();

    // Register before enabling socket notifications so events see a client.
    server->AddClient(this, sock);

    imgSock->SetEventHandler(*this, wxID_ANY);
    imgSock->SetNotify(wxSOCKET_LOST_FLAG);
    imgSock->Notify(true);
}

ImageFrameClientHandler::~ImageFrameClientHandler()
{
    delete[] imgBuffer;
}

void ImageFrameClientHandler::Destroy()
{
    Disconnect(wxEVT_SOCKET, wxSocketEventHandler(ImageFrameClientHandler::OnSocketEvent), NULL, this);

    // Remove from server state before destroying the socket.
    imgServer->RemoveClient(this);

    if (imgSock)
    {
        imgSock->Destroy();
        imgSock = nullptr;
    }
    CallAfter([this]() { delete this; });
}

void ImageFrameClientHandler::ResetState()
{
    hdr.dataLength = 0;
    bytesReceived = 0;
    headerReceived = false;
    m_partialStartMs = 0;
}

long long ImageFrameClientHandler::IdleMs() const
{
    long long now = ::wxGetUTCTimeMillis().GetValue();
    long long last = m_lastByteAtMs.load(std::memory_order_relaxed);
    long long diff = now - last;
    return diff > 0 ? diff : 0;
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

    // Let the OS eventually surface dead-without-FIN peers.
    int keepalive = 1;
    imgSock->SetOption(SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

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

        // Tear down wedged partial transfers instead of waiting for socket loss.
        if (m_partialStartMs != 0)
        {
            long long now = ::wxGetUTCTimeMillis().GetValue();
            long long elapsed = now - m_partialStartMs;
            long long budget = headerReceived
                ? (long long) FRAME_MONITOR_FRAME_TIMEOUT_MS
                : (long long) FRAME_MONITOR_HEADER_TIMEOUT_MS;
            if (elapsed > budget)
            {
                Debug.Write(wxString::Format(
                    FRAME_MONITOR_CAMERA ": partial-receive watchdog: %s "
                    "incomplete after %lld ms (budget %lld ms), closing\n",
                    headerReceived ? "frame" : "header", elapsed, budget));
                imgServer->StopClient(true);
                break;
            }
        }

        if (!headerReceived)
        {
            imgSock->Read(header + bytesReceived, headerSize - bytesReceived);
            uint32_t got = imgSock->LastReadCount();
            // Update partial-transfer and idle-eviction timestamps.
            if (got > 0)
            {
                long long now = ::wxGetUTCTimeMillis().GetValue();
                m_lastByteAtMs.store(now, std::memory_order_relaxed);
                if (m_partialStartMs == 0)
                    m_partialStartMs = now;
            }
            bytesReceived += got;

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
            // Re-arm the watchdog for the frame body.
            m_partialStartMs = ::wxGetUTCTimeMillis().GetValue();
            continue;
        }

        uint32_t limit = wxMin(FRAME_IMAGE_BUFFER_SIZE, hdr.dataLength);
        if (bytesReceived < limit)
        {
            imgSock->Read(imgBuffer + bytesReceived, limit - bytesReceived);
            uint32_t got = imgSock->LastReadCount();
            if (got > 0)
                m_lastByteAtMs.store(::wxGetUTCTimeMillis().GetValue(),
                                     std::memory_order_relaxed);
            bytesReceived += got;
        }

        if (bytesReceived == hdr.dataLength)
        {
            ProcessImage();
            ResetState();   // also clears m_partialStartMs
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

    // Single-client policy: wait for stopping clients, evict idle ones,
    // and reject a second active client.
    if (client)
    {
        if (IsClientStopping())
        {
            while (!stop_flag && WaitClientStopped(100))
                ;
        }
        else
        {
            long long idle = client->IdleMs();
            if (idle > FRAME_MONITOR_IDLE_EVICTION_MS)
            {
                Debug.Write(wxString::Format(
                    FRAME_MONITOR_CAMERA ": evicting idle client "
                    "(idle %lld ms > %d ms threshold) to accept new "
                    "connection\n", idle, FRAME_MONITOR_IDLE_EVICTION_MS));
                StopClient(true);
                while (!stop_flag && WaitClientStopped(100))
                    ;
            }
            else
            {
                Debug.Write(wxString::Format(
                    FRAME_MONITOR_CAMERA ": rejecting new connection; "
                    "existing client still active (idle %lld ms)\n", idle));
                socket->Destroy();
                return;
            }
        }
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
    SetThreadName("PHD2 FrameMon Server");
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
            // Keep client/clientSock stable while the worker reads frames.
            wxCriticalSectionLocker locker(imgServer->clientsLock);
            ImageFrameClientHandler *cli = imgServer->client;
            wxSocketBase *sock = imgServer->clientSock;
            if (cli && sock && sock->IsConnected() && !imgServer->IsClientStopping())
            {
                imgServer->connected_flag = true;
                if (sock->WaitForRead(0, 100))
                    cli->ReadFrame();
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

bool CameraFrameMonitor::Capture(usImage& img, const CaptureParams& captureParams)
{
    int duration = captureParams.duration;
    int options = captureParams.captureOptions;

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

namespace {

// Decimated dims used to fit a w x h 16-bit frame into a slot (uniform, aspect-preserving).
// decW==w, decH==h when it already fits.
static void ComputeDecimatedDims(int w, int h, int& decW, int& decH)
{
    decW = w;
    decH = h;
    if (w <= 0 || h <= 0)
        return;
    if ((size_t) w * h * 2 > SHM_FRAME_SLOT_BYTES)
    {
        double s = std::sqrt((double) ((size_t) w * h * 2) / (double) SHM_FRAME_SLOT_BYTES);
        decW = (int) (w / s);
        decH = (int) (h / s);
        while (decW > 1 && decH > 1 && (size_t) decW * decH * 2 > SHM_FRAME_SLOT_BYTES)
        {
            decW = (decW * 63) / 64;
            decH = (decH * 63) / 64;
        }
    }
}

class ShmFrameChannel
{
public:
    ShmFrameChannel()
        : m_hMap(nullptr), m_hReady(nullptr), m_shm(nullptr), m_mapSize(0), m_nextSlot(0), m_counter(0)
    {
    }
    ~ShmFrameChannel() { Close(); }

    bool Open(int instance)
    {
        wxCriticalSectionLocker lock(m_lock);
        if (m_shm)
            return true;

        m_mapSize = SHM_FRAME_PIXELS_OFFSET + (size_t) SHM_FRAME_SLOTS * SHM_FRAME_SLOT_BYTES;
        wxString mapName = wxString::Format("Local\\phd2-solar.%d.frame", instance);
        wxString evtName = wxString::Format("Local\\phd2-solar.%d.frameReady", instance);

        uint64_t mapSize64 = (uint64_t) m_mapSize;
        m_hMap = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, (DWORD) (mapSize64 >> 32),
                                    (DWORD) (mapSize64 & 0xffffffffu), mapName.wc_str());
        if (!m_hMap)
        {
            Debug.Write(wxString::Format("FrameExport: CreateFileMapping failed (%d)\n", (int) GetLastError()));
            return false;
        }
        bool existed = (GetLastError() == ERROR_ALREADY_EXISTS);

        m_shm = (ShmFrameLayout *) MapViewOfFile(m_hMap, FILE_MAP_ALL_ACCESS, 0, 0, m_mapSize);
        if (!m_shm)
        {
            Debug.Write(wxString::Format("FrameExport: MapViewOfFile failed (%d)\n", (int) GetLastError()));
            CloseInternal();
            return false;
        }

        if (!existed)
        {
            memset((void *) m_shm, 0, sizeof(ShmFrameLayout));
            m_shm->magic = SHM_FRAME_MAGIC;
            m_shm->version = SHM_FRAME_VERSION;
            m_shm->slotBytes = (uint32_t) SHM_FRAME_SLOT_BYTES;
            m_shm->slotCount = SHM_FRAME_SLOTS;
            m_shm->publishedSlot = -1;
        }

        m_hReady = CreateEventW(nullptr, FALSE, FALSE, evtName.wc_str());
        if (!m_hReady)
        {
            Debug.Write(wxString::Format("FrameExport: CreateEvent failed (%d)\n", (int) GetLastError()));
            CloseInternal();
            return false;
        }

        m_nextSlot = 0;
        Debug.Write(wxString::Format("FrameExport: opened %s (%llu bytes)\n", mapName, (unsigned long long) m_mapSize));
        return true;
    }

    void Close()
    {
        wxCriticalSectionLocker lock(m_lock);
        CloseInternal();
    }

    uint32_t Publish(const unsigned short *px, int w, int h, int bpp, int binning, int expMs, double pxUm)
    {
        wxCriticalSectionLocker lock(m_lock);
        if (!m_shm || !px || w <= 0 || h <= 0)
            return 0;

        // Decimate uniformly (preserving aspect ratio) so an oversized frame fits the
        // slot. HM maps its detected coordinates back to full frame via orig/buffer dims.
        int decW, decH;
        ComputeDecimatedDims(w, h, decW, decH);
        if (decW < 1 || decH < 1)
            return 0;
        const unsigned short *srcData = px;
        cv::Mat decimated;
        if (decW != w || decH != h)
        {
            cv::Mat full(h, w, CV_16UC1, (void *) px);
            cv::resize(full, decimated, cv::Size(decW, decH), 0, 0, cv::INTER_AREA);
            if (!decimated.isContinuous())
                decimated = decimated.clone();
            srcData = (const unsigned short *) decimated.data;
        }

        size_t bytes = (size_t) decW * decH * 2;

        int slot = m_nextSlot;
        m_nextSlot ^= 1;

        InterlockedIncrement((volatile LONG *) &m_shm->seq[slot]);

        ShmFrameMeta& m = m_shm->meta[slot];
        m.dataLength = (uint32_t) bytes;
        m.width = (uint16_t) decW;
        m.height = (uint16_t) decH;
        m.origWidth = (uint16_t) w;
        m.origHeight = (uint16_t) h;
        m.binning = (uint16_t) binning;
        m.exposureMs = (uint16_t) (expMs > 0 ? expMs : 0);
        m.bitsPerPixel = (uint16_t) bpp;
        m.reserved = 0;
        m.pixelSize = (uint32_t) (pxUm > 0 ? pxUm * 1e6 + 0.5 : 0);
        m.frameCounter = ++m_counter;
        m.timestamp = ::wxGetUTCTimeMillis().GetValue() / 1000.0;

        memcpy((char *) m_shm + SHM_FRAME_PIXELS_OFFSET + (size_t) slot * m_shm->slotBytes, srcData, bytes);

        InterlockedIncrement((volatile LONG *) &m_shm->seq[slot]);
        InterlockedExchange((volatile LONG *) &m_shm->publishedSlot, slot);
        SetEvent(m_hReady);
        return m.frameCounter;
    }

private:
    void CloseInternal()
    {
        if (m_shm)
        {
            UnmapViewOfFile((void *) m_shm);
            m_shm = nullptr;
        }
        if (m_hMap)
        {
            CloseHandle(m_hMap);
            m_hMap = nullptr;
        }
        if (m_hReady)
        {
            CloseHandle(m_hReady);
            m_hReady = nullptr;
        }
    }

    wxCriticalSection m_lock;
    HANDLE m_hMap;
    HANDLE m_hReady;
    ShmFrameLayout *m_shm;
    size_t m_mapSize;
    int m_nextSlot;
    uint32_t m_counter;
};

static ShmFrameChannel s_frameChannel;
static std::atomic<bool> s_frameExportEnabled{ false };
static std::atomic<uint32_t> s_lastFrame{ 0 };

} // namespace

bool FrameExport::Available()
{
    return true;
}

bool FrameExport::Enable(bool on)
{
    if (on)
        s_frameExportEnabled = s_frameChannel.Open(wxGetApp().GetInstanceNumber());
    else
    {
        s_frameExportEnabled = false;
        s_frameChannel.Close();
    }
    Debug.Write(wxString::Format("FrameExport: %s\n", s_frameExportEnabled.load() ? "enabled" : "disabled"));
    return on ? s_frameExportEnabled.load() : true;
}

bool FrameExport::IsEnabled()
{
    return s_frameExportEnabled.load();
}

uint32_t FrameExport::CurrentFrame()
{
    return s_lastFrame.load();
}

double FrameExport::ExportScale(int w, int h)
{
    if (w <= 0 || h <= 0)
        return 1.0;
    int decW, decH;
    ComputeDecimatedDims(w, h, decW, decH);
    if (decW < 1)
        return 1.0;
    return (double) w / (double) decW;
}

void FrameExport::Publish(const unsigned short *pixels, int width, int height, int bitsPerPixel, int binning,
                          int exposureMs, double pixelSizeUm)
{
    if (!s_frameExportEnabled.load())
        return;
    uint32_t frame = s_frameChannel.Publish(pixels, width, height, bitsPerPixel, binning, exposureMs, pixelSizeUm);
    if (frame)
        s_lastFrame = frame;
}

#else

wxString GetFrameMonitorLabel()
{
    return wxString();
}

bool FrameExport::Available() { return false; }
bool FrameExport::Enable(bool) { return false; }
bool FrameExport::IsEnabled() { return false; }
uint32_t FrameExport::CurrentFrame() { return 0; }
double FrameExport::ExportScale(int, int) { return 1.0; }
void FrameExport::Publish(const unsigned short *, int, int, int, int, int, double) { }

#endif // FRAME_MONITOR_CAMERA
