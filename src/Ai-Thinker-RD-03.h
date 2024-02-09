/**
  Ai-Thinker-RD-03 library v 0.7
  Name: Ai-Thinker-RD-03
  Purpose: Arduino library for the Ai-Thinker RD-03 24Ghz FMCW radar sensor 

  @author Dejan Gjorgjevikj
  @version 0.7, 2/2024

  This sensor is a Frequency Modulated Continuous Wave radar, ...
 
 ...todo...

Known limitations:
...

Resources:
  The code in this library was developed from scratch based on the manufacturer datasheet(s) 
  ...
  
History of changes:
    01.02.2024 - v0.1.0 initial ...
    09.02.2024 - v0.7.0 

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.
Released under LGPL-2.1 see https://github.com/Gjorgjevikj/... for full license

https://github.com/Gjorgjevikj/...todo
*/


#pragma once

#ifndef _AI_THINKER_RD_03_H
#define _AI_THINKER_RD_03_H
#include <Arduino.h>

#define AI_THINKER_RD_03_LIB_VERSION 0.7

#ifndef _LOG_LEVEL 
#define _LOG_LEVEL 2
#endif

//#define LOG_FATAL(...) Serial.printf(__VA_ARGS__)
#if _LOG_LEVEL > 0
#define R_LOG_ERROR(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_ERROR(...) { }
#endif
#if _LOG_LEVEL >= 2
#define R_LOG_WARN(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_WARN(...) { }
#endif
#if _LOG_LEVEL >= 4
#define R_LOG_INFO(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_INFO(...) { }
#endif
#if _LOG_LEVEL >= 6
#define R_LOG_DEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_DEBUG(...) { }
#endif
#if _LOG_LEVEL >= 7
#define DEBUG_DUMP_FRAME(...) _dumpFrame(__VA_ARGS__)
#else
#define DEBUG_DUMP_FRAME(...) { }
#endif
#if _LOG_LEVEL > 7
#define R_LOG_VERBOSE(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_VERBOSE(...) { }
#endif

// This library is made for little endian systems only!
#define TEST_LITTLE_ENDIAN (((union { unsigned x; unsigned char c; }){1}).c)
#ifdef TEST_LITTLE_ENDIAN

// large buffer needen for the radar debuggin mode when it send raw doppler data, not needen for the other modes of operation
#define RECEIVE_BUFFER_SIZE 2800

class AiThinker_RD_03
{
public:
    enum FrameType : uint8_t 
    { 
        UNIDENTIFIED_FRAME = 0x00,
        ENGINEERING_DATA = 0x01,
        BASIC_DATA = 0x02,
        DEBUGGING_DATA = 0x03,
        UNKNOWN_DATA_FORMAT = 0x0f,
        ACK_FRAME = 0xff,
        UNKNOWN_FRAME = 0xfe
    };
    enum RadarCommand : uint16_t 
    {
        READ_FIRMWARE_VERSION = 0x0000,    // read  the firmware version
        WRITE_REGISTER = 0x0001,    // ??
        READ_REGISTER = 0x0002,    // ??
        CONFIGURE_PARAMETER = 0x0007, // Configure ABD parameters
        READ_PARAMETER = 0x0008,
        READ_SERIAL_NUMBER = 0x0011,  // ?? does not work on v1.5.4
        SET_MODE = 0x0012,
        READ_SYSTEM_PARAMETER = 0x0013, // ??
        ENTER_FACTORY_TEST_MODE = 0x0024,
        EXIT_FACTORY_TEST_MODE = 0x0025,
        CLOSE_COMMAND_MODE = 0x00FE,      // disable configuration mode
        OPEN_COMMAND_MODE = 0x00FF       // enable configuration mode
    };

    enum CommandParameter : uint16_t // parameters for command READ_PARAMETER 
    {
        MIN_DETECTION_DISTANCE  = 0x0000,   // Minimum detection distance gate 0x0000
        MAX_DETECTION_DISTANCE  = 0x0001,   // Maximum detection distance gate 0x0001
        MIN_FRAMES_TO_DETECT    = 0x0002,   // Minimum number of target frames for detected 0x0002
        MIN_FRAMES_TO_DISSAPEAR = 0x0003,   // Minimum number of frames when the target disappears 0x0003
        TARGET_DISSAPEAR_DELAY  = 0x0004,   // Target disappearance delay time 0x0004
        GATE_HIGH_TRESHOLD      = 0x0010,   // 0x0010 - 0x001f high treashold for gate 0-15       
        GATE_LOW_TRESHOLD       = 0x0020    // 0x0020 - 0x002f low treashold for gate 0-15
    };

    enum RadarMode : uint8_t
    {
        DEBUGGING_MODE = 0x00, // Debugging mode - reporting RDMap
        REPORTING_MODE = 0x04, // Reporting mode - reports the energy value and detection result of each distance gate
        OPERATING_MODE = 0x64    // Running mode - prints output status
//        COMMAND_MODE = 0xff    // Command mode - waits and responds to configuration commands
    };

    enum FrameField : uint32_t
    {
        CMD_FRAME_HEADER  = 0xFAFBFCFD,
        CMD_FRAME_TRAILER = 0x01020304,
        ACK_FRAME_HEADER = 0xFAFBFCFD,
        ACK_FRAME_TRAILER = 0x01020304,
        DAT_FRAME_HEADER = 0xF1F2F3F4,
        DAT_FRAME_TRAILER = 0xF5F6F7F8,
        DBG_FRAME_HEADER = 0x1410BFAA,
        DBG_FRAME_TRAILER = 0xFAFBFCFD
    };
    enum FrameHeadByte : uint8_t
    {
        ACK_FRAME_HEAD_BYTE = 0xfd,
        DAT_FRAME_HEAD_BYTE = 0xf4,
        DBG_FRAME_HEAD_BYTE = 0xaa
    };

    enum FrameCL : bool
    {
        CURRENT_FRAME = false,
        LAST_FRAME = true
    };

#pragma pack (1)
    // frame header / trailer 
    union FrameMarkerType
    {
        uint32_t frameWord;
        uint8_t frameBytes[4];
    };
    // for identifying the frame type and getting the length
    struct FrameStart
    {
        FrameMarkerType header;
        uint16_t ifDataLength;
//        uint8_t modeType; // if dataframe - 0x01 Enginnering data, 0x02 Target basic information
    };
    struct REQframeCommand
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength;
        uint16_t commandWord;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };
    struct REQframeCommandWithValue
    {
        FrameMarkerType header; // FD FC FB FA == 0xfafbfcfd
        uint16_t ifDataLength; // 04 00 == 0x0004
        uint16_t commandWord; // FF 00 == 0x00ff
        uint16_t commandValue; // 01 00 == 0x0001
        FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
    };
    struct REQframeCommandValueWithPar
    {
        FrameMarkerType header; // FD FC FB FA == 0xfafbfcfd
        uint16_t ifDataLength; // 08 00 == 0x0008
        uint16_t commandWord; // 07 00 == 0x0007
        uint16_t commandPar; // 0x0000 - 0x0004
        uint32_t commandValue; //
        FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
    };
    struct ACKframeCommand
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength; // 04 00 == 0x0004
        uint16_t commandReply;
        uint16_t ackStatus;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };
    struct ACKframeCommandModeEnter
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength; // 08 00 == 0x0004
        uint16_t commandReply; // FF 01 == 0x01ff
        uint16_t ackStatus;
        uint16_t protocolVer;
        uint16_t bufferSize;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

    struct REQframeSetSystemMode
    {
        FrameMarkerType header; // FD FC FB FA == 0xfafbfcfd
        uint16_t ifDataLength; // 08 00 == 0x0008
        uint16_t commandWord; // 12 00 == 0x0012 - SET_MODE
        uint16_t parWord; // 00 00 == 0x0000 
        uint32_t parValue; // 0x00 / 0x04 / 0x64
        FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
    };

    /// <summary>
/// struct representing the firmware version of the radar sensor (contains the firmware type, major, minor and bugfix parts of the version)
/// </summary>
    //struct FirmwareVersion
    //{
    //    uint16_t type;    // firmware type
    //    uint8_t minor;    // minor version of the radar firmware
    //    uint8_t major;    // major version of the radar firmware
    //    uint32_t bugfix;  // bug fix version of the radar firmware
    //};
    struct FirmwareVersion // 8 bytes
    {
        uint16_t len;    // firmware type
        char verStr[6]; // version in string format
    };

    struct ACKframeFirmwareVersion
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength;
        uint16_t commandReply;
        uint16_t ackStatus;
        FirmwareVersion version;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

    struct ACKframeParameter
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength; // 08 00 == 0x0008
        uint16_t commandReply; // FF 01 == 0x01ff
        uint16_t ackStatus; // 0 / 1
        uint32_t parValue; 
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

    struct ACKframeSerialNum
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength; // 08 00 == 0x0008
        uint16_t commandReply; // FF 01 == 0x01ff
        uint16_t moduleID; 
        uint32_t serial;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

    struct ReportingFrame // RD-01 uses little-endian format
    {
        FrameMarkerType header; // F4 F3 F2 F1 0xf1f2f3f4
        uint16_t ifDataLength; //0x23
        uint8_t targetState; // 0 / 1
        uint16_t targetRange; // 
        uint16_t rangeEnergy[16]; // Each range gate energy
        FrameMarkerType trailer; // F8 F7 F6 F5 0xf5f6f7f8
    };

    struct DebuggingFrame
    {
        FrameMarkerType header; // AA BF 10 14 0x1410bfaa
        // 20 * 16 * 4 bytes (20 Doppler, 16 ranges, 4 bytes for each point, data value of 4 bytes per point)
        uint32_t doppler[20][16]; //doppler[320]; 
        FrameMarkerType trailer; // FD FC FB FA 0xfafbfcfd
    };

    struct FactoryData
    {
        uint16_t BoardModel;    // Daughter board model: reserved, fill in 0
        uint16_t CascadeChips;  // Number of chips : 1 - single chip, 2 - double chip, , ,
        uint16_t Channels;      // Number of channels : number of receiving channels
        uint16_t DataType;      // Data type : 0 - 1DFFT, 1 - 2DFFT, 2 - 2DFFT PEAK, 3 - DSRAW
        uint16_t DFFTsize;      //
        uint16_t ChirpsPerFrame;//
        uint16_t downsampling;  // Downsampling rate : 1 means no downsampling
    };

    struct AckFactoryTest
    {
        FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
        uint16_t ifDataLength;  // 16 ??
        uint16_t commandReply;  // 24 01 == 0x0124
        FactoryData factoryData;
        FrameMarkerType trailer; // 04 03 02 01 0x01020304
    };

 #pragma pack (0)

public:
    
    /// <summary>
    /// Class representing the radar sensor 
    /// communicates to the radar through the UART
    /// </summary>
    AiThinker_RD_03();

    /// <summary>
    /// Initialize the radar sensor and associate it to UART 
    /// </summary>
    /// <param name="rStream">Stream object representing the UART</param>
    /// <remarks>Has to be initialized to he appropriate speed before calling begin()</remarks>
    /// <returns>true on sucess</returns>
    bool begin(HardwareSerial& rStream, int rxPin, int txPin, int rxBufferSize = 2560);

    /// <summary>
    /// Sets the timeout for waiting for an accknowledgement frame
    /// </summary>
    /// <param name="timeout">Time in ms to wait for a response (ACK fame)</param>
    void setFrameTimeOut(unsigned long timeout) { frameTimeOut = timeout; }

    /// <summary>
    /// Returns the timeout for waiting for an accknowledgement frame in milliseconds
    /// </summary>
    /// <returns>Time in milliseconds to wait for a response (ACK fame)</returns>
    unsigned long getFrameTimeOut() const { return frameTimeOut; }

    /// <summary>
    /// Sets the minimum time between issuing two successive commands to the radar
    /// </summary>
    /// <param name="interComDelay">Time in milliseconds between two successive commands</param>
    /// <remarks>Some radars are slower in procceing issued comands and have small imput buffers (RD-01)</remarks>
    void setInterCommandDelay(unsigned long interComDelay) { interCommandDelay = interComDelay; }

    /// <summary>
    /// Returns the minimum time between issuing two successive commands to the radar
    /// </summary>
    /// <returns>Time in milliseconds between two successive commands</returns>
    unsigned long getInterCommandDelay() const { return interCommandDelay; }

    /// <summary>
    /// Reads the data coming from the radar sensor
    /// </summary>
    /// <remarks>Reads the data coming from the radar about the sensed target and stores it in a buffer</remarks>
    /// <remarks>To be called repeaditly in the loop()</remarks>
    /// <returns>number of bytes read</returns>
    int read();

    /// <summary>
    /// Returns the timestamp (millis) of the current/last frame 
    /// </summary>
    /// <remarks>can select current or previous frame</remarks>
    /// <param name="frame">frame selector: true (default) for the last cmoplitely received frame / false for the current frame</param>
    /// <returns>timstamp (millis) at which the first byte of the frame has been received</returns>
    unsigned long frameStartMillis(bool frame = true) const { return frame ? lastFrameStartTS : currentFrameStartTS; } // time start of last frame by default

    /// <summary>
    /// Returns the type of the current/last frame 
    /// </summary>
    /// <remarks>can select current or previous frame</remarks>
    /// <param name="frame">frame selector: true (default) for the last cmoplitely received frame / false for the current frame</param>
    /// <returns>FrameType value  
    /// <list type="bullet">
    /// <item>UNIDENTIFIED_FRAME, </item>
    /// <item>ENGINEERING_DATA, </item>
    /// <item>BASIC_DATA, </item>
    /// <item>UNKNOWN_DATA_FORMAT, </item>
    /// <item>ACK_FRAME, </item>
    /// <item>UNKNOWN_FRAME </item>
    /// </list>
    /// </returns>
    FrameType frameType(bool frame = AiThinker_RD_03::LAST_FRAME) const; // type of the last completely received frame by default

    /// <summary>
    /// Enter the configuration mode of the radar sesnsor
    /// </summary>
    /// <remarks>Sends the command to the sensor to enable configuration mode and waits for reply (ACK frame) from the sensor</remarks>
    /// <returns>true on sucess</returns>
    bool enableConfigMode();

    /// <summary>
    /// Exit the configuration mode of the radar sesnsor
    /// </summary>
    /// <remarks>Sends the command to the sensor to end configuration mode</remarks>
    /// <returns>true on sucess</returns>
    bool disableConfigMode();

    bool isInConfigMode() const { return inConfigMode; }

    /// <summary>
    /// Returns the system mode of operation
    /// </summary>
    /// <remarks>If called while the radar is in command mode will retur the operationg mode when the radar will exit the command mode</remarks>
    /// <returns>the operating mode of the radar: one of DEBUGGING_MODE / REPORTING_MODE / OPERATING_MODE</returns>
    RadarMode getSystemMode() const { return opMode; }

    /// <summary>
    /// Sets the working mode of the radar 
    /// </summary>
    /// <param name="sysMode">mode to set: OPERATING_MODE, REPORTING_MODE or DEBUGGING_MODE</param>
    /// <returns>true on sucess</returns>
    bool setSystemMode(RadarMode sysMode);

    int32_t reqParameter(uint16_t par, uint16_t cmd = READ_PARAMETER);

    int32_t getMinDetectionDistance() { return reqParameter(MIN_DETECTION_DISTANCE); }
    int32_t getMaxDetectionDistance() { return reqParameter(MAX_DETECTION_DISTANCE); }
    int32_t getMinFramesForDetection() { return reqParameter(MIN_FRAMES_TO_DETECT); }
    int32_t getMinFramesForDisappear() { return reqParameter(MIN_FRAMES_TO_DISSAPEAR); }
    int32_t getTargetDisappearDelay() { return reqParameter(TARGET_DISSAPEAR_DELAY); }

    int32_t getHighTreashold(int gate) { return reqParameter(GATE_HIGH_TRESHOLD + gate); }
    int32_t getLowTreashold(int gate) { return reqParameter(GATE_LOW_TRESHOLD + gate); }

    bool setHighTreashold(int gate, uint32_t val) { return sendCommand(CONFIGURE_PARAMETER, GATE_HIGH_TRESHOLD + gate, val); }
    bool setLowTreashold(int gate, uint32_t val) { return sendCommand(CONFIGURE_PARAMETER, GATE_LOW_TRESHOLD + gate, val); }

    // does not work
    uint16_t getModuleId(uint32_t &serial);
    FactoryData enterFactoryTestMode();
    bool exitFactoryTestMode() { return sendCommand(EXIT_FACTORY_TEST_MODE); }
    int32_t getSystemParameter(uint16_t par) { return reqParameter(par, READ_SYSTEM_PARAMETER); }

    /// <summary>
    /// Sets the minimum detection distance of the radar  
    /// </summary>
    /// <param name="gate">distance gate 0-15</param>
    /// <returns>true on sucess</returns>
    bool setMinDetectionDistance(uint32_t gate) { return sendCommand(CONFIGURE_PARAMETER, MIN_DETECTION_DISTANCE, gate); }

    /// <summary>
    /// Sets the maximim detection distance of the radar  
    /// </summary>
    /// <param name="gate">distance gate 0-15</param>
    /// <returns>true on sucess</returns>
    bool setMaxDetectionDistance(uint32_t gate) { return sendCommand(CONFIGURE_PARAMETER, MAX_DETECTION_DISTANCE, gate); }

    /// <summary>
    /// Sets the minimum number of frames with target for presence detection
    /// </summary>
    /// <param name="nFrames">number of frmaes</param>
    /// <returns>true on sucess</returns>
    /// <returns>I belive this is the number of consecutive frames with target presence detected in order to register presence</returns>
    bool setMinFramesForDetection(uint32_t nFrames) { return sendCommand(CONFIGURE_PARAMETER, MIN_FRAMES_TO_DETECT, nFrames); }

    /// <summary>
    /// Sets the minimum number of frames without target for target disappearance 
    /// </summary>
    /// <param name="nFrames">number of frmaes</param>
    /// <returns>true on sucess</returns>
    /// <returns>I belive this is the number of consecutive frames without target presence detected in order to register target disappearance</returns>
    bool setMinFramesForDisappear(uint32_t nFrames) { return sendCommand(CONFIGURE_PARAMETER, MIN_FRAMES_TO_DISSAPEAR, nFrames); }

    /// <summary>
    /// Sets the target disappearance delay time 
    /// </summary>
    /// <param name="delay">delay time</param>
    /// <returns>true on sucess</returns>
    bool setTargetDisappearDelay(uint32_t delay) { return sendCommand(CONFIGURE_PARAMETER, TARGET_DISSAPEAR_DELAY, delay); }

    const char * getFirmwareVersion();

    uint16_t getProtocolVersion(uint16_t & bufferSize);

    /// <summary>
    /// Returns data about the targets
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>target state: encoded as movingTarget(0/1), stationaryTarget(0/1) existance in the 2 LSB</returns>
    uint8_t target() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetState; }

    /// <summary>
    /// Returns the distance to the moving target registered by the radar sensor
    /// </summary>
    /// <remarks>Note: returns the data from the last completely received packet (that can be up to 70ms old)</remarks>
    /// <returns>the distance (in cm)</returns>
    uint16_t targetDistance() const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->targetRange; }

    /// <summary>
    /// Returns the energy of the moving target at given distance
    /// </summary>
    /// <param name="n">the number of the gate 0-15 (default ??cm per gate)</param>
    /// <returns>energy (0-100)</returns>
    uint16_t energyAtDistance(int n) const { return (reinterpret_cast<const ReportingFrame*>(bufferLastFrame))->rangeEnergy[n]; }

    /// <summary>
    /// Returns the doppler map bin values 
    /// </summary>
    /// <param name="row">row in the map (speed?) </param>
    /// <param name="col">column in the map (range?) </param>
    /// <returns>bin value of the coeficient</returns>
    /// <remarks>Should be called only whil the radar is working in DEBUGGING_MODE</remarks>
    uint32_t doppler(int i, int j) const { return (reinterpret_cast<const DebuggingFrame*>(bufferLastFrame))->doppler[i][j]; }
    
    /// <summary>
    /// Returns the length in bytes of th last received frame
    /// </summary>
    /// <returns>the length in bytes</returns>
    int frameLength() const { return receivedFrameLen; }

//private:
// left public for debugging, low level controll of the radar an working with unsupported protocols

    /// <summary>
    /// Sends raw data to the sensor
    /// </summary>
    /// <param name="data">pointer to the data buffer</param>
    /// <param name="size">number of butes to send</param>
    void write(const uint8_t* data, int size); //send raw data

    /// <summary>
    /// Waits for the response from the radar (ACK frame) after issuing a command to the radar
    /// </summary>
    /// <remarks>Reads the incoming data of the ACK frame and stores it in the buffer and checks its validity</remarks>
    /// <returns>true on sucess, false on timeout</returns>
    bool readAckFrame();

    /// <summary>
    /// Issues a command to the radar
    /// </summary>
    /// <remarks>Sends the given command to the radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <returns>true on sucess</returns>
    bool sendCommand(RadarCommand command);

    /// <summary>
    /// Issues a command with parameter to the radar
    /// </summary>
    /// <remarks>Sends the given command + parameter to the radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <param name="value">value fo the parameter</param>
    /// <returns>true on sucess</returns>
    bool sendCommand(RadarCommand command, uint16_t value);

    /// <summary>
    /// Issues a command with parameter and value to the radar
    /// </summary>
    /// <remarks>Sends the given command + command parameter + value to the radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <remarks>If the sensor was already in command mode before calling this function it will remain in command mode</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <param name="par">command parameter to be sent (see <see cref="enum CommandParameter">enum CommandParameter</see>)</param>
    /// <param name="value">value of the parameter</param>
    /// <returns>true on sucess</returns>
    bool sendCommand(RadarCommand cmd, uint16_t par, uint32_t value);

    /// <summary>
    /// Waits for an accknoledgement for an issued command
    /// </summary>
    /// <remarks>Waits for an accknoledgement for an issued command to radar sensor waits for the ACK frame and checks its validity</remarks>
    /// <param name="command">command to be sent (see <see cref="enum RadarCommand">enum RadarCommand</see>)</param>
    /// <param name="AckFrameReceived">was an ACK frame succesfully received</param>
    /// <param name="remainInConfig">wherethere to remain on config mode after ACK has been received</param>
    /// <returns>true on sucess</returns>
    bool commandAccknowledged(RadarCommand command, bool AckFrameReceived, bool remainInConfig);

    /// <summary>
    /// DEBUGGING: Dumps the butes of the last complitely received frame 
    /// </summary>
    void _dumpLastFrame(String label = "", Stream& dumpStream = Serial) const {
        DEBUG_DUMP_FRAME(bufferLastFrame, 
            inConfigMode ? (reinterpret_cast<const FrameStart*>(bufferLastFrame))->ifDataLength + 10 : 2,
            label + "[", "]", dumpStream);
    }

    /// <summary>
    /// DEBUGGING: Dumps the butes of the last current (ongoing) frame 
    /// </summary>
    void _dumpCurrentFrame(String label = "", Stream& dumpStream = Serial) const {
        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, label + "[", "]", dumpStream);
    }

    /// <summary>
    /// DEBUGGING: Dumps a buffer as a frame 
    /// </summary>
    static void _dumpFrame(const uint8_t* buff, int len, String pre = "", String post = "", Stream& dumpStream = Serial);

    /// <summary>
    /// Returns a pointer to the frmae buffer in memory (of the current or last frame)
    /// </summary>
    /// <remarks>current frame still holds the frame received before the last one, until the current frame begins to overwrite it</remarks>
    /// <param name="frame">frame selector: true (default) for the last complitely received frame / false for the current frame</param>
    /// <returns>pointer to the memory buffer where the frame is stored</returns>
    const char* lastFrame(bool frame = true) const { return reinterpret_cast<const char *>(frame ? bufferLastFrame : bufferCurrentFrame); }

    void clearRxBuffer();

private:
    void init();
    int readUntilHeader(FrameField header);
    void swapBuffers();

    HardwareSerial* radarUART;
    uint8_t receiveBuffer[2][RECEIVE_BUFFER_SIZE];
    uint8_t* bufferLastFrame; // pointer to the buffer where the last finished frame is stored
    uint8_t* bufferCurrentFrame; // pointer to the buffer where the next (or currently beeing recived) frame is to be stored
    unsigned long currentFrameStartTS; // the timestamp (milliseconds) when the first bute of the current frame has been recived
    unsigned long lastFrameStartTS; // the timestamp (milliseconds) when the first bute of the last finished frame has been recived
    unsigned long frameTimeOut; // user settable - timeout in milliseconds to wait for reply (ACK frame) after issuing a commnad
    unsigned long interCommandDelay; // user settable - minimum time between issued commands to the radar (req for RD-01)
    int receivedFrameLen; // the actual length of the last received frame 
    int currentFrameIndex; // the index to the buffer position wher the next received byte is going to be written
    RadarMode opMode; // system operational mode one of command / running / reporting / debugging
    bool inConfigMode; // is tha radar sensor in config mode
    bool inFrame; // has the receiving of the frame begun
    bool frameReady; // is the frame finished (all bytes recived of the current frame)
};

#else
#error "This library is made for little endian systems only, and it seems it is beeing compiled for big endian system!" 
#endif //TEST_LITTLE_ENDIAN

#endif //_AI_THINKER_RD_03_H
