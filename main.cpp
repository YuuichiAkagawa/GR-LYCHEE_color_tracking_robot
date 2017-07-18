/* 
 * GR-LYCHEE demo - 赤色の物体を検出し追いかけるロボット (YUV版)
 * Copyright (c) 2017 Yuuichi Akagawa
 *
 */
#define DBG_CAPTURE (0)
#define DBG_PCMONITOR (0)
#define FEATURE_ESP32MONITOR (1)
#define FEATURE_FOLLOW (1)

#if (DBG_CAPTURE == 1) && (FEATURE_FOLLOW == 1)
#error Both DBG_CAPTURE and FEATURE_FOLLOW can not be specified.
#endif

#if (DBG_PCMONITOR == 1) && (FEATURE_ESP32MONITOR == 1)
#error Both DBG_PCMONITOR and FEATURE_ESP32MONITOR can not be specified.
#endif

#include "mbed.h"
#include "dcache-control.h"
#include "EasyAttach_CameraAndLCD.h"
#include "opencv.hpp"
#if (DBG_CAPTURE == 1)
#include "SdUsbConnect.h"
#endif
#if (DBG_PCMONITOR == 1)
#include "DisplayApp.h"
#endif
#include "Labeling.hpp"
#include "I2CMotorDriver.h"
#include "JPEG_Converter.h"

using namespace cv;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

//ESP32 control pins
DigitalOut esp32_en(P5_3);
DigitalOut esp32_io0(P3_14);

/******************************************************************************
 * Camera Settings
 ******************************************************************************/
#define VIDEO_FORMAT (DisplayBase::VIDEO_FORMAT_YCBCR422)
#define GRAPHICS_FORMAT (DisplayBase::GRAPHICS_FORMAT_YCBCR422)
#define WR_RD_WRSWA (DisplayBase::WR_RD_WRSWA_32_16BIT)
#define DATA_SIZE_PER_PIC (2u)
#define VIDEO_PIXEL_HW (320u) /* QVGA */
#define VIDEO_PIXEL_VW (240u) /* QVGA */

#define FRAME_BUFFER_STRIDE (((VIDEO_PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT (VIDEO_PIXEL_VW)

#if defined(__ICCARM__)
#pragma data_alignment = 32
static uint8_t FrameBuffer_Video[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT] @".mirrorram";
#else
static uint8_t FrameBuffer_Video[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT] __attribute((section("NC_BSS"), aligned(32)));
#endif

/******************************************************************************
 * JPEG Settings 
 ******************************************************************************/
#define SIDE_OF_JPEGBUF 2
#define JPEGBUF_SIZE    (1024 * 20)

#if defined(__ICCARM__)
#pragma data_alignment = 32
static uint8_t JpegBuffer[SIDE_OF_JPEGBUF][JPEGBUF_SIZE];
#else
static uint8_t JpegBuffer[SIDE_OF_JPEGBUF][JPEGBUF_SIZE] __attribute((aligned(32)));
#endif
static JPEG_Converter Jcu;
static DisplayBase Display;

size_t encode_jpeg(uint8_t *buf, int len, int width, int height, uint8_t *inbuf)
{
    size_t encode_size;
    JPEG_Converter::bitmap_buff_info_t bitmap_buff_info;
    JPEG_Converter::encode_options_t encode_options;
    bitmap_buff_info.width = width;
    bitmap_buff_info.height = height;
    bitmap_buff_info.format = JPEG_Converter::WR_RD_YCbCr422;
    bitmap_buff_info.buffer_address = (void *)inbuf;
    encode_options.encode_buff_size = len;
    encode_options.p_EncodeCallBackFunc = NULL;
    encode_options.input_swapsetting = JPEG_Converter::WR_RD_WRSWA_32_16_8BIT;

    encode_size = 0;
    dcache_invalid(buf, len);
    if (Jcu.encode(&bitmap_buff_info, buf, &encode_size, &encode_options) != JPEG_Converter::JPEG_CONV_OK)
    {
        encode_size = 0;
    }

    return encode_size;
}
size_t create_jpeg(Mat &img, uint8_t side=0)
{
    return encode_jpeg(JpegBuffer[side], SIDE_OF_JPEGBUF, VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, img.data);
}

void set_jpeg_quarity(const uint8_t qual)
{
    Jcu.SetQuality(qual);
}

void camera_start(void)
{
    // Initialize the background to black
    for (int i = 0; i < sizeof(FrameBuffer_Video); i += 2)
    {
        FrameBuffer_Video[i + 0] = 0x10;
        FrameBuffer_Video[i + 1] = 0x80;
    }

    // Camera
    EasyAttach_Init(Display);

    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)FrameBuffer_Video,
        FRAME_BUFFER_STRIDE,
        VIDEO_FORMAT,
        WR_RD_WRSWA,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW);
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

void create_mat(cv::Mat &img_orig)
{
    // Transform buffer into OpenCV matrix
    img_orig = Mat(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC2, FrameBuffer_Video);
}

/******************************************************************************
 * Labeling settings
 ******************************************************************************/
#define DETECT_MIN_THRESHOLD 3
#define DETECT_MAX_THRESHOLD 180
/* Application variables */
Mat frame_orig;
Mat frame_bgr;
#if (DBG_CAPTURE == 1)
Mat frame_bgro;
#endif
LabelingBS labeling;
RegionInfoBS *ri;
uint8_t mask[VIDEO_PIXEL_HW * VIDEO_PIXEL_VW];             //ラベリング用マスク
short *mask2 = new short[VIDEO_PIXEL_HW * VIDEO_PIXEL_VW]; //ラベリング出力先

typedef union {
    uint32_t yuv;
    uint8_t d[4];
} _yuv422pack;

typedef union {
    uint32_t bgra;
    uint8_t d[4];
} _cv8u3;

//出力用画像バッファ(YUV422)
uint8_t yuv422buf[VIDEO_PIXEL_HW * VIDEO_PIXEL_VW * DATA_SIZE_PER_PIC];
void bgr2yuv422(cv::Mat &img_src, uint8_t *dst);

#if (FEATURE_ESP32MONITOR == 1)
//RawSerial esp32(D1, D0); //(P3_15, P0_2);
RawSerial esp32(P3_15, P0_2);
#endif

#if (DBG_PCMONITOR == 1)
/* For viewing image on PC */
static DisplayApp display_app;
#endif

#if (DBG_CAPTURE == 1)
InterruptIn button0(USER_BUTTON0);

static uint8_t shoot = 0;
static void button0_rise(void)
{
    shoot = 1;
}
#endif

/******************************************************************************
 * ESP32 settings
 ******************************************************************************/
typedef struct {
    uint8_t wp;
    uint8_t flag[SIDE_OF_JPEGBUF]; //b0:W b1:R
} jpegbuf_mgr_t;
jpegbuf_mgr_t jbm;

#if (FEATURE_ESP32MONITOR == 1)
static uint8_t isPreview = 0;
InterruptIn button1(USER_BUTTON1);
static void button1_rise(void)
{
    (isPreview == 0) ? isPreview = 1 : isPreview = 0;
    led3 = isPreview;
}

void SendHeader(uint32_t size)
{
    uint8_t headder_data[12] = {0xFF, 0xFF, 0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    headder_data[8] = (uint8_t)((uint32_t)size >> 0);
    headder_data[9] = (uint8_t)((uint32_t)size >> 8);
    headder_data[10] = (uint8_t)((uint32_t)size >> 16);
    headder_data[11] = (uint8_t)((uint32_t)size >> 24);
    for (int i = 0; i < sizeof(headder_data); i++)
    {
        esp32.putc(headder_data[i]);
    }
}

void SendData(uint8_t *buf, uint32_t size)
{
    while (size > 0)
    {
        esp32.putc(*buf++);
        size--;
    }
}
int SendJpeg(uint8_t *buf, uint32_t size)
{
    SendHeader(size);
    SendData(buf, size);
    return size;
}

typedef struct {
    uint8_t rp;
    uint8_t *buf;
    size_t size;
} esp32Msg_t;
MemoryPool<esp32Msg_t, 16> esp32Pool;
Queue<esp32Msg_t, 16> esp32Q;
Thread esp32Thread;

void esp32Task()
{
    while(1){
        osEvent evt = esp32Q.get();
        if (evt.status == osEventMessage) {
            esp32Msg_t *m = (esp32Msg_t*)evt.value.p;
            SendJpeg(m->buf, m->size);
            jbm.flag[m->rp] = 0;
            esp32Pool.free(m);
        }
    }
}
#endif

/******************************************************************************
 * Motor control settings
 ******************************************************************************/
#if (FEATURE_FOLLOW == 1)
I2CMotorDriver Motor(D14, D15, 0x0f); // sda, scl
typedef struct
{
    int motor1;
    int motor2;
} motorMessage_t;
MemoryPool<motorMessage_t, 16> motorPool;
Queue<motorMessage_t, 16> motorQ;
static int lastDrive = 0;

Thread motorThread;

InterruptIn button0(USER_BUTTON0);

static uint8_t isMotorEnabled = 0;
static uint8_t isMotorEnabledtoStop = 0;
static void button0_rise(void)
{
    if (isMotorEnabled == 0)
    {
        isMotorEnabled = 1;
    }
    else
    {
        isMotorEnabledtoStop = 1;
        isMotorEnabled = 0;
    }
    led2 = isMotorEnabled;
}

void motorTask()
{
    while (1)
    {
        osEvent evt = motorQ.get();
        if (evt.status == osEventMessage)
        {
            //led2 = 1;
            motorMessage_t *m = (motorMessage_t *)evt.value.p;
            Motor.speed(MOTOR1, m->motor1);
            Motor.speed(MOTOR2, m->motor2);
            motorPool.free(m);
            //led2 = 0;
        }
    }
}

void follow(int x1, int x2)
{
    //Motor direction
    // M1:left  - forward, + backward
    // M2:right + forward, - backward

    //motor control enabled check
    if (isMotorEnabled == 0)
    { //Disable?
        if (isMotorEnabledtoStop == 1)
        { //turn to disable?
            motorMessage_t *m = motorPool.alloc();
            m->motor1 = 0;
            m->motor2 = 0;
            motorQ.put(m);
            isMotorEnabledtoStop = 0;
        }
        return;
    }

    int maxspeed = 50;
    int L, R, w;
    if (x1 < 0 && x2 < 0)
    {
        L = R = 0;
        w = VIDEO_PIXEL_HW;
    }
    else
    {
        L = x1 - 0;
        R = VIDEO_PIXEL_HW - x2;
        w = x2 - x1;
    }

    int m1, m2, put;
    put = 0;
    int d = L - R;
    if (d < 0)
        d = -d;

    // too near or small stop all motors
    if (w > DETECT_MAX_THRESHOLD || w < DETECT_MIN_THRESHOLD)
    {
        if (lastDrive > 0)
        {
            m1 = 0;
            m2 = 0;
            put = 1;
            lastDrive = 0;
        }
    }
    else
    {
        if (d > 20)
        {
            if (L > R)
            {
                //turn right(1)
                if (lastDrive != 1)
                {
                    m1 = -maxspeed;
                    m2 = 0;
                    put = 1;
                    lastDrive = 1;
                }
            }
            else
            {
                //turn left(2)
                if (lastDrive != 2)
                {
                    m1 = 0;
                    m2 = maxspeed;
                    put = 1;
                    lastDrive = 2;
                }
            }
        }
        else
        {
            //forward(3)
            if (lastDrive > 0)
            {
                m1 = -maxspeed;
                m2 = maxspeed;
                put = 1;
                lastDrive = 3;
            }
        }
    }

    if (put > 0)
    {
        motorMessage_t *m = motorPool.alloc();
        m->motor1 = m1;
        m->motor2 = m2;
        motorQ.put(m);
    }
}
#endif

/******************************************************************************
 * マスク用色検出処理
 ******************************************************************************/
bool detectColor(uint8_t y, uint8_t u, uint8_t v)
{
    if (y <= 160 && 170 <= v && u <= 115)
        return true;
    else
        return false;
}

/******************************************************************************
 * main()
 ******************************************************************************/
int main()
{
#if (DBG_CAPTURE == 1)
    char file_name[32];
    int file_name_index_detected = 1;
    button0.rise(&button0_rise);
#endif

#if (FEATURE_ESP32MONITOR == 1)
    memset(&jbm, 0, sizeof(jpegbuf_mgr_t));

    esp32.set_flow_control(SerialBase::Disabled);
    esp32.baud(115200);
    esp32_io0 = 1;
    esp32_en = 0;
    Thread::wait(100);
    esp32_en = 1;
    Thread::wait(1000);

    //truncate ESP32 kernel boot message
    while (esp32.readable())
    {
        esp32.getc();
    }

    esp32.baud(1000000);
    button1.rise(&button1_rise);
    esp32Thread.start(esp32Task);
#endif
    //Set JPEG quairty
    set_jpeg_quarity(60);

    // Camera
    camera_start();
#if (DBG_CAPTURE == 1)
    // SD & USB
    SdUsbConnect storage("storage");
    printf("Finding a storage...");
    // wait for the storage device to be connected
    storage.wait_connect();
#endif
    printf("done\n\r");
    led1 = 1;

#if (FEATURE_FOLLOW == 1)
    //start I2C motor control thread
    motorThread.start(motorTask);
    button0.rise(&button0_rise);
#endif

    while (1)
    {
#if (FEATURE_ESP32MONITOR == 1)
        while (esp32.readable())
        {
            esp32.getc();
        }
#endif
        //Camera capture
        create_mat(frame_orig);
        if (frame_orig.empty())
        {
            printf("ERR: There is no input frame, retry to capture\n");
            continue;
        }

        // 色検出でマスク画像の作成
        uint32_t *mask32 = (uint32_t *)mask;
        uint32_t *srcimg32 = (uint32_t *)frame_orig.data;
        for (int x = 0; x < (VIDEO_PIXEL_VW * VIDEO_PIXEL_HW * 2); x += 8)
        {
            _yuv422pack data0, data1;
            data0.yuv = *srcimg32++;
            data1.yuv = *srcimg32++;
            uint8_t y0 = data0.d[0];
            uint8_t u0 = data0.d[1];
            uint8_t v0 = data0.d[3];
            uint8_t y1 = data1.d[0];
            uint8_t u1 = data1.d[1];
            uint8_t v1 = data1.d[3];
            uint32_t c = 0;

            //1st pixel
            if (detectColor(y0, u0, v0))
            {
                c = 0xffff;
            }
            else
            {
                c = 0;
            }
            //2nd pixel
            if (detectColor(y1, u1, v1))
            {
                c = c | 0xffff0000;
            }
            else
            {
                c = c & 0x0000ffff;
            }
            *mask32++ = c;
        }
        // 白領域が無い場合のエラー処理
        //rectangle(mask, Point(0, 0), Point(1, 1), Scalar(255), -1);
        mask[0] = 255;
        mask[1] = 255;
        mask[VIDEO_PIXEL_HW] = 255;
        mask[VIDEO_PIXEL_HW + 1] = 255;

#if (DBG_PCMONITOR == 1) || (FEATURE_ESP32MONITOR == 1)
        cvtColor(frame_orig, frame_bgr, COLOR_YUV2BGR_YUY2);
#endif
        //ラベリング処理
        labeling.Exec((uchar *)mask, mask2, VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, true, 30);
        if (labeling.GetNumOfResultRegions() > 0)
        {
            led4 = 1;
            //最大の領域が処理対象
            ri = labeling.GetResultRegionInfo(0);
            int x1, y1, x2, y2;
            ri->GetMin(x1, y1);
            ri->GetMax(x2, y2);
            printf("Detect: %d, x1=%d,y1=%d,x2=%d,y2=%d\r\n", labeling.GetNumOfResultRegions(), x1, y1, x2, y2);
#if (DBG_PCMONITOR == 1) || (FEATURE_ESP32MONITOR == 1)
            //対象の領域を四角で囲む
            rectangle(frame_bgr, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 2);
#endif
//            }
#if (FEATURE_FOLLOW == 1)
            //drive motors
            follow(x1, x2);
#endif
        }
        else
        {
            rectangle(frame_bgr, Point(0, 0), Point(1, 1), Scalar(0, 0, 255), 1);
#if (FEATURE_FOLLOW == 1)
            follow(-1, -1);
#endif
            led4 = 0;
        }

#if (DBG_CAPTURE == 1)
        if (shoot == 1)
        {
            cvtColor(frame_orig, frame_bgro, COLOR_YUV2BGR_YUY2);
            sprintf(file_name, "/storage/org_%d.bmp", file_name_index_detected);
            imwrite(file_name, frame_bgro);
            sprintf(file_name, "/storage/bgr_%d.bmp", file_name_index_detected);
            imwrite(file_name, frame_bgr);
            sprintf(file_name, "/storage/mask_%d.bmp", file_name_index_detected);
            Mat img_mask(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC1, mask);
            imwrite(file_name, img_mask);
            sprintf(file_name, "/storage/yuv_%d.bin", file_name_index_detected);
            FILE *fp = fopen(file_name, "w");
            fwrite(frame_orig.data, (unsigned int)(240 * 320 * 2), 1, fp);
            fclose(fp);
            file_name_index_detected++;
            shoot = 0;
        }
#endif

#if (DBG_PCMONITOR == 1)
        //preview
        bgr2yuv422(frame_bgr, yuv422buf);
        Mat img_yuv(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC2, yuv422buf);
        size_t jpeg_size = create_jpeg(img_yuv);
        display_app.SendJpeg(JpegBuffer[0], jpeg_size);
#endif

#if (FEATURE_ESP32MONITOR == 1)
        //size_t jpeg_size = create_jpeg(img_yuv);
        if (isPreview != 0) {
            bgr2yuv422(frame_bgr, yuv422buf);
            Mat img_yuv(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC2, yuv422buf);
            if(jbm.flag[jbm.wp] == 0){
                jbm.flag[jbm.wp] = 1;
                esp32Msg_t *m = esp32Pool.alloc();
                if( m != NULL ){
                    m->rp = jbm.wp;
                    m->size = create_jpeg(img_yuv, jbm.wp);
                    m->buf = JpegBuffer[jbm.wp];
                    esp32Q.put(m);
                    jbm.wp = (jbm.wp == 1) ? 0 : 1;
                }
            }
        }
#endif
    }
}

inline uint8_t clip(int32_t val)
{
    if (val > 255)
        return 255;
    if (val < 0)
        return 0;
    return(uint8_t)val;
}

/**
 * Converting BGR888 to YUV422 (8-bit BT.601)
 */
void bgr2yuv422(cv::Mat &img_src, uint8_t *dst)
{
    int32_t y0, y1, u0, v0;
    int32_t r1, g1, b1;
    int32_t r2, g2, b2;
    int32_t width, height, bytes;
    width = img_src.size().width;
    height = img_src.size().height;
    bytes = width * height * 3;

    //2ピクセル同時に処理する
    uint32_t *src32;
    uint32_t *dst32 = (uint32_t *)dst;
    _yuv422pack yuv;
    _cv8u3 bgr0, bgr1;
    for (int32_t cnt = 0; cnt < bytes; cnt += 6)
    {
        //色成分を取得
        src32 = (uint32_t *)(img_src.data + cnt + 0);
        bgr0.bgra = *src32;
        src32 = (uint32_t *)(img_src.data + cnt + 3);
        bgr1.bgra = *src32;
        b1 = (int32_t)bgr0.d[0];
        g1 = (int32_t)bgr0.d[1];
        r1 = (int32_t)bgr0.d[2];
        b2 = (int32_t)bgr1.d[0];
        g2 = (int32_t)bgr1.d[1];
        r2 = (int32_t)bgr1.d[2];
        //輝度成分の計算
        y0 = (( 66 * r1 + 129 * g1 + 25 * b1 + 128 ) >> 8) + 16;
        y1 = (( 66 * r2 + 129 * g2 + 25 * b2 + 128 ) >> 8) + 16;

        //色差成分の計算(cb = u, cr = v)
        u0 = ( ( -38 * r1 -  74 * g1 + 112 * b1 + 128) >> 8) + 128;
        v0 = ( ( 112 * r1 -  94 * g1 -  18 * b1 + 128) >> 8) + 128;

        //y0
        yuv.d[0] = clip(y0);
        //y1
        yuv.d[2] = clip(y1);
        //u0
        yuv.d[1] = clip(u0);
        //v0
        yuv.d[3] = clip(v0);

        //store
        *dst32++ = yuv.yuv;
    }
}
