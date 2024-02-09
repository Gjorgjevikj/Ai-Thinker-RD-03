/*
    Name:       AiThRD03.ino
    Created:	01.2.2024 02:38:13
    Author:     DESKTOP-I9\Dejan
*/


#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define rSerial Serial1

#define RADAR_SERIAL_SPEED 115200
#define CONSOLE_SERIAL_SPEED 115200

#define RADAR_RX_PIN 26
//#define RADAR_RX_PIN 32
#define RADAR_TX_PIN 27
//#define RADAR_TX_PIN33

#define PUSH_BUTTON1 0

#include <yaPushButton.h>
#include <Ai-Thinker-RD-03.h>

#define OPR_MODE
//#define REP_MODE
//#define DBG_MODE

//#define PLOT_ADPT
//#define TEST_SETTING_F

//#define RESENDRAW


union lng
{
    char ch[2560];
    uint32_t ulng[640];
} buf;

AiThinker_RD_03 radar;

unsigned int last_frame_ts = 0;

bool emode = false;
unsigned long t, loopStart;

char rbuf[2400];
int p = 0;
unsigned long int fs;
char rmodeBuffer[32];
int bp = 0;

char target[10] = "?";
int range = -1;
int plotOffset = 0;

bool defButtonState = 0;
bool blink1 = true;
unsigned long debounce;

PushButton<> Button(PUSH_BUTTON1);
AiThinker_RD_03::RadarMode modes[] = { AiThinker_RD_03::DEBUGGING_MODE, AiThinker_RD_03::REPORTING_MODE, AiThinker_RD_03::OPERATING_MODE };
int cmode = 2;


unsigned long last_mode_change = 0;

typedef int32_t(AiThinker_RD_03::* fpGetPar)();
typedef bool (AiThinker_RD_03::* fpSetPar)(uint32_t);

void changeBack(AiThinker_RD_03& rad, fpGetPar getPar, fpSetPar setPar, const char* name)
{
    Serial.print("Get "); Serial.print(name);
    int32_t rb, nd, dd = (rad.*getPar)();
    if (dd < 0)
        Serial.println(" ### FAILED ###");
    else
    {
        Serial.print(" = "); Serial.println(dd);
    }

    nd = dd + 1; nd %= 16;

    Serial.print("Set "); Serial.print(name);
    Serial.print(" to "); Serial.print(nd);
    if (!(rad.*setPar)(nd))
        Serial.println(" failed.");
    else
        Serial.println(" sucess.");
    delay(200);

    Serial.print("... read back ");
    rb = (rad.*getPar)();
    if (rb < 0)
        Serial.println("### FAILED ###");
    else
    {
        Serial.print(rb); Serial.print(rb == nd ? " = " : " != "); Serial.println(nd);
    }
    delay(200);

    Serial.print("... set back ");
    if (!(rad.*setPar)(dd))
        Serial.println(" failed.");
    else
        Serial.println(" sucess.");
    delay(200);

    Serial.print("... check back ");
    rb = (rad.*getPar)();
    if (rb < 0)
        Serial.println("### FAILED ###");
    else
    {
        Serial.print(rb); Serial.print(rb == dd ? " = " : " != "); Serial.println(dd);
    }
    delay(200);
}

void setup()
{
    Serial.begin(CONSOLE_SERIAL_SPEED); //Feedback over Serial Monitor
    //rSerial.begin(RADAR_SERIAL_SPEED, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar
    //rSerial.setRxBufferSize(1400);
    //pinMode(PUSH_BUTTON1, INPUT_PULLUP);
    Button.init();

    Serial.println("...will start in 2s.");
    delay(2000);

    radar.begin(rSerial, RADAR_RX_PIN, RADAR_TX_PIN);

    // if found in other mode ...
    //radar.setFrameTimeOut(500);
    //radar.setInterCommandDelay(100);
    //radar.clearRxBuffer();
    //while(!radar.enableConfigMode())
    //    Serial.print(".");
    //while(!radar.setSystemMode(AiThinker_RD_03::OPERATING_MODE))
    //    Serial.print(",");
    //delay(100);
    //radar.clearRxBuffer();
    //radar.disableConfigMode();

    radar.setFrameTimeOut(200);
    radar.setInterCommandDelay(100);


    //Serial.printf("sizeof(ReportingFrame)=%d\n", sizeof(HLK_LD2410::ReportingFrame));
    //Serial.printf("sizeof(EngineeringFrame)=%d\n", sizeof(HLK_LD2410::EngineeringFrame));

    long int dd, nd, x;

    // Firmware
    const char* firmvareVersion = radar.getFirmwareVersion();
    uint16_t bufferSize, protocolVersion = radar.getProtocolVersion(bufferSize);
    delay(500);
    Serial.println("\nRadar sensor Ai-Thinker RD-03 test");
    delay(2000);
    Serial.print("firmware ");
    Serial.print(firmvareVersion);
    if (protocolVersion)
    {
        Serial.print(" protocol v");
        Serial.print(protocolVersion);
        Serial.print(" bufferSize=");
        Serial.println(bufferSize);
    }
    delay(200);

#ifdef TEST_SETTING_F

    Serial.println("Starting Config mode...");
    if (!radar.enableConfigMode())
        Serial.println("Entering config mode failed.");
    else
        Serial.println("Entering config mode sucess.");
    Serial.println("In config mode...");
    delay(500);

    int maxdd, mindd, fdet, fdis, tdd;

    Serial.print("MinDetectionDistance ");
    mindd = radar.getMinDetectionDistance();
    if (mindd < 0)
        Serial.println("### FAILED ###");
    else
        Serial.print("= "); Serial.println(mindd);

    Serial.print("MaxDetectionDistance ");
    maxdd = radar.getMaxDetectionDistance();
    if (maxdd < 0)
        Serial.println("### FAILED ###");
    else
        Serial.print("= "); Serial.println(maxdd);

    Serial.print("MinFramesForDetection ");
    fdet = radar.getMinFramesForDetection();
    if (fdet < 0)
        Serial.println("### FAILED ###");
    else
        Serial.print("= "); Serial.println(fdet);

    Serial.print("MinFramesForDisappear ");
    fdis = radar.getMinFramesForDisappear();
    if (fdis < 0)
        Serial.println("### FAILED ###");
    else
        Serial.print("= "); Serial.println(fdis);

    Serial.print("TargetDisappearDelay ");
    tdd = radar.getTargetDisappearDelay();
    if (tdd < 0)
        Serial.println("### FAILED ###");
    else
        Serial.print("= "); Serial.println(tdd);

    // test set/get parameters
    Serial.println("---------------");
    changeBack(radar, &AiThinker_RD_03::getMinDetectionDistance, &AiThinker_RD_03::setMinDetectionDistance, "MinDetectionDistance");
    Serial.println();
    changeBack(radar, &AiThinker_RD_03::getMaxDetectionDistance, &AiThinker_RD_03::setMaxDetectionDistance, "MaxDetectionDistance");
    Serial.println();
    changeBack(radar, &AiThinker_RD_03::getMinFramesForDetection, &AiThinker_RD_03::setMinFramesForDetection, "MinFramesForDetection");
    Serial.println();
    changeBack(radar, &AiThinker_RD_03::getMinFramesForDisappear, &AiThinker_RD_03::setMinFramesForDisappear, "MinFramesForDisappear");
    Serial.println();
    changeBack(radar, &AiThinker_RD_03::getTargetDisappearDelay, &AiThinker_RD_03::setTargetDisappearDelay, "TargetDisappearDelay");
    Serial.println();

    Serial.println("Exit Config mode...");
    if (!radar.disableConfigMode())
        Serial.println("Exiting config mode failed.");
    else
        Serial.println("Exiting config mode sucess.");
    delay(2500);

#endif

    /*
    Serial.println("Try to read Version info...");
    //AiThinker_RD_03::FirmwareVersion ver;
    const char * fwv = radar.getFirmwareVersion();
    //char version[12] = { 0 };
    //if(ver.len<11)
    //    strncpy(version, ver.verStr, ver.len);
    //else
    //    strncpy(version, ver.verStr, 11);
    Serial.printf("Version %s\n", fwv);
    delay(200);
    */
    /*
        int32_t tsv = radar.getHighTreashold(10);
        radar.setHighTreashold(10, 280);
        Serial.println("Treasholds for gates:");
        for (int i = 0; i < 16; i++)
        {
            Serial.printf("Gate %2d low: %3d high: %3d\n",i,
                radar.getLowTreashold(i),
                radar.getHighTreashold(i));
        }
        radar.setHighTreashold(10, tsv);
    */

    /*
    // It does not work.... it seems unsupported commanda on this version
    uint32_t serial;
    uint16_t mid;
    mid = radar.getModuleId(serial);
    Serial.printf("Module ID: %04x seial: %d\n", mid, serial);
    delay(200);

    Serial.print("enterFactoryTestMode()... ");
    AiThinker_RD_03::FactoryData fd;
    fd = radar.enterFactoryTestMode();
    delay(200);
    Serial.print("exitFactoryTestMode() ");
    if(radar.exitFactoryTestMode())
        Serial.println("successfull");
    else
        Serial.println("### FAILED ###");
    delay(200);

    Serial.print("getSystemParameter ");
    int32_t spar = radar.getSystemParameter(1);
    if (spar < 0)
        Serial.println("### FAILED ###");
    else
        Serial.print("= "); Serial.println(spar);
    */

    /*
        Serial.print("getProtocolVersion ");
        uint16_t bSize=0, pVer;
        pVer = radar.getProtocolVersion(bSize);
        if (pVer)
        {
            Serial.print(pVer); Serial.print(" bufferSize="); Serial.println(bSize);
        }
        else
            Serial.println("### FAILED ###");
        delay(200);


        Serial.println("Exit Config mode...");
        if (!radar.disableConfigMode())
            Serial.println("Exiting config mode failed.");
        else
            Serial.println("Exiting config mode sucess.");
        delay(2500);
        */

    Serial.print("Setting mode to "); //ascii
#if defined(OPR_MODE)
    Serial.print("OPERATING_MODE "); //ascii
    if (!radar.setSystemMode(AiThinker_RD_03::OPERATING_MODE))
        Serial.println("failed.");
    else
        Serial.println("sucess.");
#elif defined(REP_MODE)
    Serial.println("REPORTING_MODE ");
    if (!radar.setSystemMode(AiThinker_RD_03::REPORTING_MODE))
        Serial.println("failed.");
    else
        Serial.println("sucess.");
#elif defined(DBG_MODE)
    Serial.println("DEBUGGING_MODE ");
    if (!radar.setSystemMode(AiThinker_RD_03::DEBUGGING_MODE))
        Serial.println("failed.");
    else
        Serial.println("sucess.");
#endif    
    delay(500);

    /*
    Serial.println();
    delay(500);
    Serial.println("testing...");
    Serial.print("sendCommand(READ_FIRMWARE_VERSION) : ");
    Serial.println(radar.sendCommand(AiThinker_RD_03::READ_FIRMWARE_VERSION));
    delay(500);
    Serial.print("sendCommand(RESTART) : ");
    Serial.println(radar.sendCommand(AiThinker_RD_03::RESTART));
    delay(500);
*/


    if (radar.isInConfigMode())
    {
        Serial.println("Exit Config mode...");
        if (!radar.disableConfigMode())
            Serial.println("Exiting config mode failed.");
        else
            Serial.println("Exiting config mode sucess.");
    }
    delay(500);

    switch (radar.getSystemMode())
    {
    case AiThinker_RD_03::OPERATING_MODE:
        Serial.println("In OPERATING mode...");
        break;
    case AiThinker_RD_03::REPORTING_MODE:
        Serial.println("In REPORTING mode...");
        break;
    case AiThinker_RD_03::DEBUGGING_MODE:
        Serial.println("In DEBUGGING mode...");
        Serial.print("\033[2J");
        Serial.print("\033[H");
        break;
    default:
        Serial.print("In ??? mode... = ");
        Serial.println(radar.getSystemMode(), HEX);
        break;
    }


    //Serial.println("\nSTOP");
    //while (1)
    //    delay(10);

    //Serial.println("loop...");
    //delay(500);
    //defButtonState = digitalRead(PUSH_BUTTON1);
    loopStart = last_frame_ts = millis();
    last_mode_change = millis();
}

void loop()
{
    int t = millis();

    radar.read();
    //Serial.printf("(%6lu): %s\t", radar.lastFrameStartTS, radar.lastFrame(true));
    //Serial.printf("(%6lu): %s\n", radar.currentFrameStartTS, radar.lastFrame(false));


    if (radar.getSystemMode() == AiThinker_RD_03::REPORTING_MODE)
    {
        if (last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME))
        {
            last_frame_ts = radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME);

#ifdef PLOT_ADPT
            Serial.printf("%d, %3d", radar.target() * 10, radar.targetDistance());
            plotOffset = 500;
            for (int i = 0, div = 100; i < 16; i++, div = div / 2 + 1)
            {
                Serial.printf(",%5d", radar.energyAtDistance(i) / div + plotOffset);
                plotOffset += 500;
            }
            //for (int i = 0; i < 16; i++)
            //{
            //    Serial.printf(",%.1f", sqrt(radar.energyAtDistance(i)) + plotOffset);
            //    plotOffset += 500;
            //}

#else
            Serial.printf("%d, %3d", radar.target(), radar.targetDistance());
            for (int i = 0; i < 16; i++)
            {
                Serial.printf(",%5d", radar.energyAtDistance(i));
            }
#endif
            Serial.println();
        }
    }
    else if (radar.getSystemMode() == AiThinker_RD_03::OPERATING_MODE)
    {
        if (last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME)
            && last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME))
        {
            radar._dumpCurrentFrame();
            radar._dumpLastFrame();

            last_frame_ts = max(radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME), radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME));
            //Serial.printf("(%6lu)REP: %s %s\n", 
            //    radar.frameStartMillis(), radar.lastFrame(AiThinker_RD_03::CURRENT_FRAME), radar.lastFrame(false));

            if (!sscanf(radar.lastFrame(AiThinker_RD_03::CURRENT_FRAME), "Range%d", &range))
            {
                sscanf(radar.lastFrame(AiThinker_RD_03::CURRENT_FRAME), "%4s", target);
            }
            if (!sscanf(radar.lastFrame(AiThinker_RD_03::LAST_FRAME), "Range%d", &range))
            {
                sscanf(radar.lastFrame(AiThinker_RD_03::LAST_FRAME), "%4s", target);
            }
            Serial.printf("%s : range %d\n", target, range);
        }
    }
    else if (radar.getSystemMode() == AiThinker_RD_03::DEBUGGING_MODE)
    {
        if (last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME)
            && last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME))
        {
            last_frame_ts = max(radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME), radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME));

#ifdef RESENDRAW
            Serial.write(radar.lastFrame(AiThinker_RD_03::LAST_FRAME), radar.receivedFrameLen);
#else
            if (radar.frameLength() == 1288)
            {
                //int imax = 0, jmax = 0, max=0;
                //for (int i = 0; i < 20; i++)
                //    for (int j = 0; j < 16; j++)
                //    {
                //        if (radar.doppler(i, j) > max)
                //        {
                //            max = radar.doppler(i, j);
                //            imax = i;
                //            jmax = j;
                //        }
                //    }
                char line[40];
                Serial.print("\033[H");
                for (int i = 0; i < 20; i++)
                {
                    for (int j = 0; j < 16; j++)
                    {
                        if (radar.doppler(i, j) > 100000UL)
                            line[j] = '#';
                        else if (radar.doppler(i, j) > 10000UL)
                            line[j] = '+';
                        else if (radar.doppler(i, j) > 1000UL)
                            line[j] = '.';
                        else
                            line[j] = ' ';
                    }
                    line[16] = 0;

                    //Serial.printf("%6d,", radar.doppler(i, j));
                    Serial.println(line);
                }
                Serial.println();
                //Serial.printf("max=%d @ (%d,%d)\n", max, imax, jmax);
            }
            else
                Serial.printf("DBG frame ??? len = %d\n", radar.frameLength());
#endif
        }
    }

    /*
    if (millis() - last_mode_change > 5000)
    {
        //delay(200);
        if (radar.enableConfigMode())
        {
            Serial.println("in config mode");

            switch (radar.getSystemMode())
            {
            case AiThinker_RD_03::OPERATING_MODE:
                //delay(200);
                //Serial.println("Set setSystemMode to REPORTING_MODE");
                if (!radar.setSystemMode(AiThinker_RD_03::REPORTING_MODE))
                    Serial.println("System mode set to REPORTING_MODE failed.");
                else
                    Serial.println("System mode set to REPORTING_MODE sucess.");
                break;
            case AiThinker_RD_03::REPORTING_MODE:
                //Serial.println("Set setSystemMode to DEBUGGING_MODE");
                if (!radar.setSystemMode(AiThinker_RD_03::DEBUGGING_MODE))
                    Serial.println("System mode set to DEBUGGING_MODE failed.");
                else
                    Serial.println("System mode set to DEBUGGING_MODE sucess.");
                break;
            case AiThinker_RD_03::DEBUGGING_MODE:
                if (!radar.setSystemMode(AiThinker_RD_03::OPERATING_MODE))
                    Serial.println("System mode set to OPERATING_MODE failed.");
                else
                    Serial.println("System mode set to OPERATING_MODE sucess.");
                delay(500);
            }
            Serial.print("System mode now: ");
            Serial.println(radar.getSystemMode(), HEX);
            delay(100);
            Serial.println("exit config mode");
            radar.disableConfigMode();
            last_mode_change = last_frame_ts = millis();
        }
    }
    */

    if (Button.stateChanged() == BUTTON_RELEASED)
    {
        cmode++;
        cmode %= 3;
        Serial.print("\n %% CHANGING MODE to: ");
        Serial.print(cmode);
        if (!radar.setSystemMode(modes[cmode]))
            Serial.println(" failed.");
        else
            Serial.println(" sucess.");
    }
    delay(1);
}



