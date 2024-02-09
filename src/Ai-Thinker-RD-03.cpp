#include <Ai-Thinker-RD-03.h>

// This library is made for little endian systems only!
#ifdef TEST_LITTLE_ENDIAN

AiThinker_RD_03::AiThinker_RD_03() : radarUART(nullptr), frameTimeOut(120), interCommandDelay(0), opMode(OPERATING_MODE)
{
    init();
}

void AiThinker_RD_03::init()
{
    inConfigMode = inFrame = frameReady = false;
    receivedFrameLen = currentFrameIndex = 0;
    bufferLastFrame = receiveBuffer[1];
    bufferCurrentFrame = receiveBuffer[0];
}

bool AiThinker_RD_03::begin(HardwareSerial& rSerial, int rxPin, int txPin, int rxBufferSize)
{
    radarUART = &rSerial;
    radarUART->setRxBufferSize(rxBufferSize);
    radarUART->begin(115200, SERIAL_8N1, rxPin, txPin); //UART for monitoring the radar
    init();
    return true;
}

//swap reading buffers
void AiThinker_RD_03::swapBuffers()
{
    //frameReady = true;
    uint8_t* tmp = bufferLastFrame;
    bufferLastFrame = bufferCurrentFrame;
    receivedFrameLen = currentFrameIndex;
    lastFrameStartTS = currentFrameStartTS;
    bufferCurrentFrame = tmp;
    currentFrameIndex = 0;
    //inFrame = false;
}

void AiThinker_RD_03::clearRxBuffer()
{
    unsigned long int t = millis();
    while (radarUART->available() && millis() - t < 1500)
        radarUART->read();
}

// reads incoming bytes on the serial and stores them in buffer
int AiThinker_RD_03::read()
{
    int rc = 0; // number of bytes read in this call
    if (inConfigMode)
    {
            if (!inFrame)
            {
                rc += readUntilHeader(ACK_FRAME_HEADER);
            }
            else // inFrame
            {
                while (radarUART->available())
                {
                    uint8_t c = radarUART->read();
                    rc++;
                    bufferCurrentFrame[currentFrameIndex++] = c;

                    if (currentFrameIndex >= RECEIVE_BUFFER_SIZE)
                    {
                        R_LOG_ERROR("ERROR: Receive buffer overrun!\n");
                        inFrame = false;
                        frameReady = false;
                        currentFrameIndex = 0;
                    }
                    else if (currentFrameIndex >= 14
                        && *(reinterpret_cast<const uint32_t*>(bufferCurrentFrame + (reinterpret_cast<const ACKframeCommand*>(bufferCurrentFrame))->ifDataLength + 6)) == ACK_FRAME_TRAILER)
/*                    else if ((currentFrameIndex >= 14
                        && (reinterpret_cast<const ACKframeCommand*>(bufferCurrentFrame))->ifDataLength == 4
                        && *(reinterpret_cast<const uint32_t*>(bufferCurrentFrame + 10)) == ACK_FRAME_TRAILER) // short ack
                        ||
                        (currentFrameIndex >= 18 // ack with paraemeter
                            && (reinterpret_cast<const ACKframeCommand*>(bufferCurrentFrame))->ifDataLength == 8
                            && *(reinterpret_cast<const uint32_t*>(bufferCurrentFrame + 14)) == ACK_FRAME_TRAILER)
                        ||
                        (currentFrameIndex >= 22 // version info frame
                            && (reinterpret_cast<const ACKframeCommand*>(bufferCurrentFrame))->ifDataLength == 12
                            && *(reinterpret_cast<const uint32_t*>(bufferCurrentFrame + 18)) == ACK_FRAME_TRAILER)
                        )*/
                    {
                        frameReady = true;
                        swapBuffers();
                        inFrame = false;
                        break;
                    }
                }
            }
        //} // end while(available)
    }
    else // not in config mode
    {
        if (opMode == OPERATING_MODE) // ASCII messages every ~100ms
        {
            while (radarUART->available())
            {
                uint8_t c = radarUART->read();
                if (currentFrameIndex==0)
                    currentFrameStartTS = millis();
                bufferCurrentFrame[currentFrameIndex++] = c;
                rc++;
                if (currentFrameIndex >= RECEIVE_BUFFER_SIZE)
                {
                    R_LOG_ERROR("ERROR: Receive buffer overrun!\n");
                    inFrame = false;
                    currentFrameIndex = 0;
                }
                if (c == '\n')
                {
                    bufferCurrentFrame[currentFrameIndex] = 0;
                    swapBuffers();
                }
            }
        }
        else if (opMode == REPORTING_MODE)
        {
            if (inFrame)
            {
                int ab = radarUART->available();
                if (ab)
                {
                    if (currentFrameIndex + ab > 45) // 45 == reporting frame size
                        ab = 45 - currentFrameIndex;
                    radarUART->readBytes(bufferCurrentFrame + currentFrameIndex, ab);
                    currentFrameIndex += ab;
                    rc += ab;
                }
                // assuming fixed predetermined size of data frames, 
                // does not check the frame size field, does  check for trailer??
                if (currentFrameIndex >= 45) // check for proper trailer 
                {
                    const uint32_t* trailer = reinterpret_cast<const uint32_t*>(bufferCurrentFrame + 41);
                    if (*trailer == DAT_FRAME_TRAILER)
                    {
                        frameReady = true;
                        swapBuffers();
                        inFrame = false;
                    }
                }
            }
            else // not inFrame
            {
                rc += readUntilHeader(DAT_FRAME_HEADER);
            }
        }
        else if (opMode == DEBUGGING_MODE)
        {
            if (inFrame)
            {
                int ab = radarUART->available();
                if (ab)
                {
                    if (currentFrameIndex + ab > 1288) // 1288 == debugging frame size
                        ab = 1288 - currentFrameIndex;
                    radarUART->readBytes(bufferCurrentFrame + currentFrameIndex, ab);
                    currentFrameIndex += ab;
                    rc += ab;
                }
                // assuming fixed predetermined size of data frames, 
                // does not check the frame size field, does  check for trailer??
                //if (currentFrameIndex >= 1288) // check for proper trailer 
                //const uint32_t* trailer = reinterpret_cast<const uint32_t*>(bufferCurrentFrame + 1284);
                const uint32_t* trailer = reinterpret_cast<const uint32_t*>(bufferCurrentFrame + currentFrameIndex -  4);
                if (*trailer == DBG_FRAME_TRAILER)
                {
                    frameReady = true;
                    swapBuffers();
                    inFrame = false;
                }
            }
            else // not inFrame
            {
                rc += readUntilHeader(DBG_FRAME_HEADER);
            }
        }
    } // not in config mode
    return rc;
}

int AiThinker_RD_03::readUntilHeader(FrameField header)
{
    int rc = 0;
    while (radarUART->available())
    {
        uint8_t c = radarUART->read();
        rc++;
        //if (currentFrameIndex == 0)
        //    currentFrameStartTS = millis();
        bufferCurrentFrame[currentFrameIndex++] = c;
        if (currentFrameIndex == 4) // header possibly in buffer
        {
            // check for valid header
            uint32_t* head = reinterpret_cast<uint32_t*>(bufferCurrentFrame);
            if (*head == header)
            {
                inFrame = true;
                frameReady = false;
                currentFrameStartTS = millis();
                break;
            }
            else // not valid header ... skip until header
            {
                uint8_t frameHeadByte = static_cast<uint8_t>(header & 0x000000ff);
                while (currentFrameIndex && *bufferCurrentFrame != frameHeadByte)
                {
                    currentFrameIndex--;
                    (*head) >>= 8;
                }
            }
        }
    }
    return rc;
}
 
//int AiThinker_RD_03::read()
//{
//    int rc = 0; // number of bytes read in this call
//    while (radarUART->available())
//    {
//        uint8_t c = radarUART->read();
//        rc++;
//        if (!inFrame)
//        {
//            if (opMode == DEBUGGING_MODE && !inConfigMode)
//            {
//                if (c != 0xaa)
//                    continue;
//            }
//            if (c == ACK_FRAME_HEAD_BYTE || c == DAT_FRAME_HEAD_BYTE || c == DBG_FRAME_HEAD_BYTE) // new frame begining (c>>4 == 0x0f)
//            {
//                inFrame = true;
//                frameReady = false;
//                //swap reading buffers
//                bufferLastFrame = bufferCurrentFrame;
//                receivedFrameLen = currentFrameIndex;
//                lastFrameStartTS = currentFrameStartTS;
//                bufferCurrentFrame = bufferLastFrame;
//                currentFrameIndex = 0;
//                currentFrameStartTS = millis();
//            }
//        }
//        else
//        {
//            if (currentFrameIndex > 6) // the frame type and length can be determined 
//            {
//                if (frameType(CURRENT_FRAME) == UNIDENTIFIED_FRAME)
//                {
//
//                    inFrame = frameReady = false;
//                    currentFrameIndex = 0;
//                    R_LOG_WARN("Sync!\n");
//                }
//                else if (frameType(CURRENT_FRAME) == ENGINEERING_DATA)
//                {
//                    if (currentFrameIndex >= (reinterpret_cast<const FrameStart*>(bufferCurrentFrame))->ifDataLength + 7)
//                    {
//                        frameReady = true; // the rest are trailer bytes, so the frame can be processed 
//                    }
//                }
//                else if (frameType(CURRENT_FRAME) == DEBUGGING_DATA)
//                {
//                    if (currentFrameIndex >= sizeof(DebuggingFrame) - sizeof(FrameMarkerType))
//                    {
//                        frameReady = true; // the rest are trailer bytes, so the frame can be processed 
//                    }
//                }
//            }
//            if (currentFrameIndex >= RECEIVE_BUFFER_SIZE)
//            {
//                R_LOG_ERROR("ERROR: Receive buffer overrun!\n");
//                inFrame = false;
//                currentFrameIndex = 0;
//            }
//
//        }
//
//        bufferCurrentFrame[currentFrameIndex++] = c;
//
//        if ( (frameType(CURRENT_FRAME) == ENGINEERING_DATA || frameType(CURRENT_FRAME) == ACK_FRAME)
//            && currentFrameIndex >= (reinterpret_cast<const FrameStart*>(bufferCurrentFrame))->ifDataLength + 10)
//        {
//            inFrame = false;
//        }
//        if (frameType(CURRENT_FRAME) == DEBUGGING_DATA && currentFrameIndex >= sizeof(DebuggingFrame))
//        {
//            inFrame = false;
//            R_LOG_DEBUG("currentFrameIndex=%d\n", currentFrameIndex);
//            _dumpCurrentFrame();
//        }
//
//
//
//
//    }
//    return rc;
//}


void AiThinker_RD_03::write(const uint8_t* data, int size) //send raw data
{
    static unsigned long lastCommandIssuedAt = 0UL;
    DEBUG_DUMP_FRAME(data, size, ">>[","]");
    if (interCommandDelay > 0)
    {
        unsigned long sinceLast = millis() - lastCommandIssuedAt;
        if(sinceLast < interCommandDelay)
            delay(interCommandDelay - sinceLast);
    }
    radarUART->write(data, size);
    radarUART->flush();
}

// check the type of the frame
// frame=true -> last completed frame, frame=false -> current frame
AiThinker_RD_03::FrameType AiThinker_RD_03::frameType(bool frame) const
{
    if (frame || currentFrameIndex > 6) // the basic type can be determined by the first byte only
    {
        const FrameStart* fheadid = reinterpret_cast<const FrameStart*>(frame ? bufferLastFrame : bufferCurrentFrame);
        switch (fheadid->header.frameWord)
        {
        case DAT_FRAME_HEADER:
            return ENGINEERING_DATA;
            break;
        case DBG_FRAME_HEADER:
            return DEBUGGING_DATA;
            break;
        case ACK_FRAME_HEADER:
            return ACK_FRAME;
            break;
        default:
            return UNKNOWN_FRAME;
        }
    }
    return UNIDENTIFIED_FRAME; // not yet identified
}

bool AiThinker_RD_03::readAckFrame()
{
    unsigned long now = millis();
    
    frameReady = false;
    // wait for something to arrive / frame to start
    while (!read() && millis() - now < frameTimeOut)
        delay(1);
    if (millis() - now >= frameTimeOut)
    {
        R_LOG_WARN("# TimeOut while waiting for reply - no response\n");
        return false;
    }
    
    // waiting for ACK frame
    now = millis();
    while (!frameReady && millis() - now < frameTimeOut)
    {
        if (!read())
            delay(1);
    }
    // either timeout or ACK has arrived and is in bufferLastFrame now
    if (millis() - now >= frameTimeOut)
    {
        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, "##[", "]");
        R_LOG_WARN("# TimeOut while waiting for ACK frame\n");
        return false;
    }
    else
        DEBUG_DUMP_FRAME(bufferLastFrame, receivedFrameLen, "++[", "]");

    if (frameType(LAST_FRAME) != ACK_FRAME)
    {
        R_LOG_WARN("# NOT an ACK frame!\n");
        return false;
    }
    return true;
}


//bool AiThinker_RD_03::readAckFrame() 
//{
//    unsigned long now = millis();
//
//    // wait for something to arrive / frame to start
//    while (!read() && millis() - now < frameTimeOut)
//        delay(1);
//        //yield();
//    if (millis() - now >= frameTimeOut)
//    {
//        R_LOG_WARN("# TimeOut while waiting for reply - no response\n");
//        return false;
//    }
//
//    // waiting for ACK frame, skip the other (if any) frames that were in the buffer
//    now = millis();
//    while (frameType(CURRENT_FRAME) != ACK_FRAME && millis() - now < frameTimeOut)
//    {
//        //while (!frameReady && millis() - now < frameTimeOut)
//        {
//            if (!read())
//                delayMicroseconds(50);
//        }
//    }
//    if (millis() - now >= frameTimeOut)
//    {
//        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, "##[", "]");
//        R_LOG_WARN("# TimeOut while waiting for ACK frame\n");
//        //yield();
//        return false;
//    }
//    if (frameType(CURRENT_FRAME) == ACK_FRAME)
//    {
//        int expLen = (reinterpret_cast<const FrameStart*>(bufferCurrentFrame))->ifDataLength + 10;
//        now = millis();
//        while (currentFrameIndex < expLen && millis() - now < frameTimeOut)
//        {
//            if (!read())
//                delayMicroseconds(50);
//        }
//        DEBUG_DUMP_FRAME(bufferCurrentFrame, currentFrameIndex, "==[", "]");
//        if (millis() - now >= frameTimeOut)
//        {
//            R_LOG_ERROR("# TimeOut reading ACK frame\n");
//            //yield();
//            return false;
//        }
//    }
//    else
//    {
//        R_LOG_ERROR("# TimeOut while waiting for ACK frame\n");
//        return false;
//    }
//    return true;
//}


bool AiThinker_RD_03::enableConfigMode() 
{
    // when entering config mode (from reporting mode) several reporting frames meight be already in the buffer
    // a longer timeout is needed to ignore them
    // or ... another apprach would be to clean up the incoming buffer of butes waiting to be read before issuing the command.
    const static REQframeCommandWithValue cmdEnableConfMode = { CMD_FRAME_HEADER, 0x0004, OPEN_COMMAND_MODE, 0x0001, CMD_FRAME_TRAILER };
    int retry = 2;
    while (!inConfigMode && retry--)
    {
        unsigned long t = millis();
        while (radarUART->available() && millis() - t < frameTimeOut)
            radarUART->read();
        write(reinterpret_cast<const uint8_t*>(&cmdEnableConfMode), sizeof(REQframeCommandWithValue));
        inConfigMode = true; // for readAckFrame() to expect command replay ACK
        bool ack = readAckFrame();
        inConfigMode = commandAccknowledged(OPEN_COMMAND_MODE, ack, true);
    }
    return inConfigMode;
}

bool AiThinker_RD_03::disableConfigMode() 
{
    const static REQframeCommand cmdEndConfMode = { CMD_FRAME_HEADER, 0x0002, CLOSE_COMMAND_MODE, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t *>(&cmdEndConfMode), sizeof(REQframeCommand));
    //readAckFrame();
    inConfigMode = !commandAccknowledged(CLOSE_COMMAND_MODE, readAckFrame(), true);
    return !inConfigMode;
}

uint16_t AiThinker_RD_03::getProtocolVersion(uint16_t& bufSize)
{
    bool wasInConfigMode = inConfigMode;
    uint16_t ver = 0;
    enableConfigMode();
    const static REQframeCommandWithValue cmdEnableConfMode = { CMD_FRAME_HEADER, 0x0004, OPEN_COMMAND_MODE, 0x0001, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdEnableConfMode), sizeof(REQframeCommandWithValue));
    if (readAckFrame())
    {
        const ACKframeCommandModeEnter* frame = reinterpret_cast<const ACKframeCommandModeEnter *>(bufferLastFrame);
        ver = frame->protocolVer;
        bufSize = frame->bufferSize;
    }
    if(!wasInConfigMode)
        disableConfigMode();
    return ver;
}


const char * AiThinker_RD_03::getFirmwareVersion()
{
    const uint16_t FW_STR_SIZE = 15;
    static char fwVerStr[FW_STR_SIZE+1] = "?"; // null terminated
    //static FirmwareVersion ver = { 2, "v?"};
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommand cmd = { CMD_FRAME_HEADER, 0x0002, READ_FIRMWARE_VERSION, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommand));
    bool ackValid = readAckFrame();
    const ACKframeFirmwareVersion* ack = reinterpret_cast<const ACKframeFirmwareVersion*>(bufferLastFrame);
    //ver = ack->version;
    int nc = min((ack->version).len, FW_STR_SIZE);
    strncpy(fwVerStr, (ack->version).verStr, nc);
    fwVerStr[nc] = 0;
    return commandAccknowledged(READ_FIRMWARE_VERSION, ackValid, wasInConfigMode) ? fwVerStr : "?";
}

uint16_t AiThinker_RD_03::getModuleId(uint32_t &ser)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommand cmd = { CMD_FRAME_HEADER, 0x0002, READ_SERIAL_NUMBER, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommand));
    bool ackValid = readAckFrame();
    const ACKframeSerialNum* ack = reinterpret_cast<const ACKframeSerialNum*>(bufferLastFrame);
    uint16_t id = ack->moduleID;
    ser = ack->serial;
    return commandAccknowledged(READ_FIRMWARE_VERSION, ackValid, wasInConfigMode) ? id : 0;
}

bool AiThinker_RD_03::setSystemMode(RadarMode sysMode)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeSetSystemMode cmd = { CMD_FRAME_HEADER, 0x0008, SET_MODE, 0x0000, sysMode, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeSetSystemMode));
    //readAckFrame();
    bool success = commandAccknowledged(SET_MODE, readAckFrame(), wasInConfigMode);
    if(success)
        opMode = sysMode;
    return success;
}

int32_t AiThinker_RD_03::reqParameter(uint16_t par, uint16_t readCmd) // cmd == READ_PARAMETER
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeCommandWithValue cmd = { CMD_FRAME_HEADER, 0x0004, readCmd, par, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommandWithValue));
    bool ackValid = readAckFrame();
    const ACKframeParameter* ack = reinterpret_cast<const ACKframeParameter*>(bufferLastFrame);
    int32_t dist = ack->parValue;
    return commandAccknowledged(static_cast<RadarCommand>(readCmd), ackValid, wasInConfigMode) ? dist : -1L;
}

AiThinker_RD_03::FactoryData AiThinker_RD_03::enterFactoryTestMode()
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    const static REQframeCommand cmd = { CMD_FRAME_HEADER, 0x0002, ENTER_FACTORY_TEST_MODE, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(REQframeCommand));
    bool ackValid = readAckFrame();
    const AckFactoryTest* ack = reinterpret_cast<const AckFactoryTest*>(bufferLastFrame);
    FactoryData fd = ack->factoryData;
    return commandAccknowledged(READ_FIRMWARE_VERSION, ackValid, wasInConfigMode) ? fd : ((struct FactoryData) { 0, 0, 0, 0, 0, 0, 0 });
}

bool AiThinker_RD_03::sendCommand(RadarCommand cmd)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeCommand cmdFrame = { CMD_FRAME_HEADER, 0x0002, cmd, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdFrame), sizeof(REQframeCommand));
    //bool ackValid = readAckFrame();
    return commandAccknowledged(cmd, readAckFrame(), wasInConfigMode);
}

bool AiThinker_RD_03::sendCommand(RadarCommand cmd, uint16_t value)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeCommandWithValue cmdFrame = { CMD_FRAME_HEADER, 0x0004, cmd, value, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdFrame), sizeof(REQframeCommandWithValue));
    //readAckFrame();
    return commandAccknowledged(cmd, readAckFrame(), wasInConfigMode);
}

bool AiThinker_RD_03::sendCommand(RadarCommand cmd, uint16_t par, uint32_t value)
{
    bool wasInConfigMode = inConfigMode;
    if (!inConfigMode)
        enableConfigMode();
    REQframeCommandValueWithPar cmdFrame = { CMD_FRAME_HEADER, 0x0008, cmd, par, value, CMD_FRAME_TRAILER };
    write(reinterpret_cast<const uint8_t*>(&cmdFrame), sizeof(REQframeCommandValueWithPar));
    //readAckFrame();
    return commandAccknowledged(cmd, readAckFrame(), wasInConfigMode);
}

bool AiThinker_RD_03::commandAccknowledged(RadarCommand cmd, bool AckFrameReceived, bool remainInConfig)
{
    const ACKframeCommand* ack = reinterpret_cast<const ACKframeCommand*>(bufferLastFrame);
    DEBUG_DUMP_FRAME(bufferLastFrame, ack->ifDataLength + 10, "<<[", "]");
    bool validAck = (AckFrameReceived
        && ack->header.frameWord == ACK_FRAME_HEADER)
        && (ack->commandReply == (cmd | 0x0100))
        && (ack->ackStatus == 0);
    if (!remainInConfig)
    {
        //delay(100);
        disableConfigMode();
    }
    return validAck;
}

void AiThinker_RD_03::_dumpFrame(const uint8_t* buff, int len, String pre, String post, Stream& dumpStream)
{
    dumpStream.print(pre);
    for (int i = 0; i < len; i++)
        dumpStream.printf("%02x", buff[i]);
    dumpStream.println(post);
}

#endif
