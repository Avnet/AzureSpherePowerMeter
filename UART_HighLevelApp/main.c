/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere demonstrates how to use a UART (serial port).
// The sample opens a UART with a baud rate of 115200. Pressing a button causes characters
// to be sent from the device over the UART; data received by the device from the UART is echoed to
// the Visual Studio Output Window.
//
// It uses the API for the following Azure Sphere application libraries:
// - UART (serial port)
// - GPIO (digital input for button)
// - log (messages shown in Visual Studio's Device Output window during debugging)
// - eventloop (system invokes handlers for timer events)

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/uart.h>
#include <applibs/gpio.h>
#include <applibs/log.h>
#include <applibs/eventloop.h>

// By default, this sample's CMake build targets hardware that follows the MT3620
// Reference Development Board (RDB) specification, such as the MT3620 Dev Kit from
// Seeed Studios.
//
// To target different hardware, you'll need to update the CMake build. The necessary
// steps to do this vary depending on if you are building in Visual Studio, in Visual
// Studio Code or via the command line.
//
// See https://github.com/Azure/azure-sphere-samples/tree/master/Hardware for more details.
//
// This #include imports the sample_hardware abstraction from that hardware definition.
#include <hw/sample_hardware.h>

#include "eventloop_timer_utilities.h"

/// <summary>
/// Exit codes for this application. These are used for the
/// application exit code.  They they must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm = 1,
    ExitCode_SendMessage_Write = 2,
    ExitCode_ButtonTimer_Consume = 3,
    ExitCode_ButtonTimer_GetValue = 4,
    ExitCode_UartEvent_Read = 5,
    ExitCode_Init_EventLoop = 6,
    ExitCode_Init_UartOpen = 7,
    ExitCode_Init_RegisterIo = 8,
    ExitCode_Init_OpenButton = 9,
    ExitCode_Init_ButtonPollTimer = 10,
    ExitCode_Main_EventLoopFail = 11,
    ExitCode_Init_OpenGPIO35 = 12,
    ExitCode_Init_MCP39F511PollTimer = 13
} ExitCode;

// File descriptors - initialized to invalid value
static int uartFd = -1;
static int gpioButtonFd = -1;
static int pwrMeterEnFd = -1;

// Declare a variable to drive output format
bool outputCSV = false;

EventLoop *eventLoop = NULL;
EventRegistration *uartEventReg = NULL;
EventLoopTimer *buttonPollTimer = NULL;
EventLoopTimer *MCP39F511PollTimer = NULL;

// State variables
static GPIO_Value_Type buttonState = GPIO_Value_High;

// Termination state
static volatile sig_atomic_t exitCode = ExitCode_Success;

static void TerminationHandler(int signalNumber);
static void SendUartMessage(int uartFd, const char *dataToSend, size_t bytesToSend);
static void ButtonTimerEventHandler(EventLoopTimer *timer);
static void MCP39F511PollTimerEventHandler(EventLoopTimer* timer);
static void UartEventHandler(EventLoop *el, int fd, EventLoop_IoEvents events, void *context);
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

/// <summary>
///     Helper function to send a fixed message to the UART
/// </summary>
void transmit_MCP39F511_commands(void) {

    // Define the message to send.  We are requesting a register read starting at address 0x0006 and reading 0x0C bytes
    const uint8_t messageToSend[] = { 0xA5, 0x08, 0x41, 0x0, 0x06, 0x4E, 0x0C, 0x4E };

    // We're only going to send one byte at a time to the MCP3f9F511 device, so declare an array of size 1
    uint8_t messagePart[1];

    // Setup ts to ~0.03 seconds
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 33333333;

    // The MCP39F511 does not correctly receive our message unless we send one byte at a time and insert a delay between bytes.
    for (int i = 0; i < 8; i++) {
        messagePart[0] = messageToSend[i];
        SendUartMessage(uartFd, messagePart, 1);
        nanosleep(&ts, NULL);
    }
}

/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
/// <parm name= "bytesToSend"> The number of bytes to send over the UART</parm>
static void SendUartMessage(int uartFd, const char *dataToSend, size_t bytesToSend)
{
    size_t totalBytesSent = 0;
    size_t totalBytesToSend = bytesToSend;
    int sendIterations = 0;
    while (totalBytesSent < totalBytesToSend) {
        sendIterations++;

        // Send as much of the remaining data as possible
        size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
        const char *remainingMessageToSend = dataToSend + totalBytesSent;
        ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
        if (bytesSent < 0) {
            Log_Debug("ERROR: Could not write to UART: %s (%d).\n", strerror(errno), errno);
            exitCode = ExitCode_SendMessage_Write;
            return;
        }

        totalBytesSent += (size_t)bytesSent;
    }
//    Log_Debug("Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations);
}

/// <summary>
///     Handle button timer event: if the button is pressed, send data over the UART.
/// </summary>
static void ButtonTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }

    // Check for a button press
    GPIO_Value_Type newButtonState;
    int result = GPIO_GetValue(gpioButtonFd, &newButtonState);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_ButtonTimer_GetValue;
        return;
    }

    // If the button has just been pressed, toggle the output boolean
    // The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
    if (newButtonState != buttonState) {
        if (newButtonState == GPIO_Value_Low) {

            //  Toggle the output boolean
            outputCSV = outputCSV ? false : true;

            if (outputCSV) {
                // Print the CSV header
                Log_Debug("voltage, current\n");

            }
        }
        
        buttonState = newButtonState;
    }
}

/// <summary>
///     Handle timer event: Read the Power Monitor device.
/// </summary>

static void MCP39F511PollTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }

    // Call the routine that sends commands to the power monitor device to request power data.
    transmit_MCP39F511_commands();
}


///<summary>
///		Parses received MCU data to extract data values and reports to IoT Hub as needed.
///</summary>
///<param name=",msg">A pointer to the receive data.</param>
///<param name="nLineLength">Length of received MCU data.</param>
//void MCU_ParseDataToIotHub(char *pszLine, size_t nLineLength)
void ParseMCP39F511Response(uint8_t* msg)
{

    // Define indexes into the response message.
    enum response_msg {
        response_id = 0,
        message_size = 1,
        voltage_rms = 2,
        line_frequency = 4,
        input_voltage = 6,
        power_factor = 8,
        current_rms = 10,
    };

    // Parse out the measurements from the mesage
    float fVoltage = (float)((uint16_t)(msg[voltage_rms + 1] << 8) | ((uint16_t)(msg[voltage_rms]))) / (float)100.0f;
    float fCurrent = (float)((uint32_t)(msg[current_rms + 3] << 24) | (uint32_t)(msg[current_rms + 2] << 16) | (uint32_t)(msg[current_rms + 1] << 8) | (uint32_t)(msg[current_rms])) / (float)1000.0f;
    
    if (outputCSV) {

        Log_Debug("%.3f, %.3f\n", fVoltage, fCurrent);
    }
    else {
        Log_Debug("voltage %.3f V, current %.3f mA \n", fVoltage, fCurrent);
    }
}

/// <summary>
///     Handle UART event: if there is incoming data, check to see if it's a complete message.
///     This satisfies the EventLoopIoCallback signature.
/// </summary>
static void UartEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context)
{
    
    // MCP39F511 Response codes
#define MCP_ACK 0x06
#define MCP_NACK 0x15
#define MCP_CSFAIL 0x51

// UART ring buffer size
#define RECEIVE_BUFFER_SIZE		128

    // Define the structure of the MCP39F511 response
    enum Msg_Fmt {
        HEADER = 0,
        MESSAGE_SIZE,
        COMMAND_BYTE,
        DATABYTE1,
        DATABYTE2,
        DATABYTE3,
        DATABYTE4,
        DATABYTE5,
        DATABYTE6
    };

    //static receive buffer for UART
    static char receiveBuffer[RECEIVE_BUFFER_SIZE];

    //Number of bytes in ring buffer
    static size_t nBytesInBuffer = 0;

    uint8_t* pchSegment = &receiveBuffer[nBytesInBuffer];
    uint8_t* msgPointer = &receiveBuffer[0];

    // Poll the UART and store the byte(s) behind already received bytes
    ssize_t nBytesRead = read(fd, (void*)pchSegment, RECEIVE_BUFFER_SIZE - nBytesInBuffer);
    nBytesInBuffer += (size_t)nBytesRead;

    /*
        Log_Debug("nBytesRead = %d\n", nBytesRead);

        for (int i = 0; i < nBytesRead; i++) {
            Log_Debug("%x ", msgPointer[i]);
        }
        Log_Debug("\n");
    */

    if (nBytesRead < 0) {
        Log_Debug("ERROR: Problem reading from UART: %s (%d).\n", strerror(errno), errno);
        return;
    }

    if (nBytesRead == 0) {
        return;
    }

    if (nBytesInBuffer >= RECEIVE_BUFFER_SIZE) // buffer overrun, discard content
    {
        Log_Debug("ERROR: UART reveiver buffer too small or EOL missing, discarding content!\n");
        nBytesInBuffer = 0;
        return;
    }

    // while we have data in the buffer that can be processed loop and process messages

    // We need to track when we have data, but not the complete message.
    bool messageIsComplete = true;

    do {

        // We have data.  Either process the data or bail if we don't have at least one complete message.

        // Switch on the message header byte
        switch (msgPointer[HEADER]) {

        case MCP_CSFAIL:
            Log_Debug("Message RX: MCP_CSFAIL\n");
            // consume the response byte
            msgPointer++;
            nBytesInBuffer--;
            break;

        case MCP_NACK:
            Log_Debug("Message RX: MCP_NACK\n");
            // consume the response byte
            msgPointer++;
            nBytesInBuffer--;
            break;

        case MCP_ACK: // We have a message to process!

            // Check to see if we have the entire message
            if (nBytesInBuffer >= msgPointer[MESSAGE_SIZE]) {

                //Log_Debug("Message RX: MCP_ACK\n");

                // First verify that the data is correct, use the message checkSum
                // Reset the checkSum variable
                uint8_t checkSum = 0;

                // Calculate the checksum.
                for (int i = 0; i <= msgPointer[MESSAGE_SIZE] - 1; i++) {
                    // We process all the bytes except the checksum value 
                    if (i < msgPointer[MESSAGE_SIZE] - 1) {
                        checkSum += msgPointer[i];
                    }
                }

                // Validate the message is not corrupt by comparing the calculated checkSum with
                // the provided checkSum in the message.  If it's bad just output a message and move on.
                // This data will be discarded.

                if (msgPointer[msgPointer[MESSAGE_SIZE] - 1] != checkSum) {
                    Log_Debug("WARNING: CheckSums DON'T match, expected %x  Discarding message!\n", checkSum);
                }
                else {

                    ParseMCP39F511Response(msgPointer);

                }

                // Adjust the Byte counter, subtract the message size we just processed
                nBytesInBuffer -= msgPointer[MESSAGE_SIZE];
            }
            else {
                // We don't have the entire message, set the flag so we fall out of the do/while loop.
                // When the additional data comes in over the UART we'll append it to the data we currently
                // have.
                messageIsComplete = false;
            }
            break;

        default:
            Log_Debug("WARNING: Unknown Header!\n");
            // consume the response byte
            msgPointer++;
            nBytesInBuffer--;
            break;
        }

    } while ((nBytesInBuffer > 0) && messageIsComplete);

    // If we have any data that we did not process, move the remaining bytes to begining of the receive buffer
    if (nBytesInBuffer > 0) {
        memcpy((void*)receiveBuffer, (const void*)msgPointer, nBytesInBuffer);
    }
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>ExitCode_Success if all resources were allocated successfully; otherwise another
/// ExitCode value which indicates the specific failure.</returns>
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

    // Create a UART_Config object: open the UART, set up UART event handler and configure
    // a timer to poll the power monitor device
    UART_Config uartConfig;
    UART_InitConfig(&uartConfig);
    uartConfig.baudRate = 9600;
    uartConfig.flowControl = UART_FlowControl_None;
    uartFd = UART_Open(SAMPLE_UART, &uartConfig);
    if (uartFd < 0) {
        Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_UartOpen;
    }
    uartEventReg = EventLoop_RegisterIo(eventLoop, uartFd, EventLoop_Input, UartEventHandler, NULL);
    if (uartEventReg == NULL) {
        return ExitCode_Init_RegisterIo;
    }

    // Set the poll time for 250ms
    struct timespec MCP39F511PollPeriod = { .tv_sec = 0, .tv_nsec = 250 * 1000 * 1000 };
    MCP39F511PollTimer = CreateEventLoopPeriodicTimer(eventLoop,  MCP39F511PollTimerEventHandler,
        &MCP39F511PollPeriod);
    if (MCP39F511PollTimer == NULL) {
        return ExitCode_Init_MCP39F511PollTimer;
    }

    // Open Power Meter Enable GPIO as output
    Log_Debug("Opening GPIO 35 as output.\n");
    // Drive the GPIO Low to enable the Click Board in Click site #2
    pwrMeterEnFd = GPIO_OpenAsOutput(AVNET_MT3620_SK_GPIO35, GPIO_OutputMode_PushPull, GPIO_Value_Low);
    if (pwrMeterEnFd < 0) {
        Log_Debug("ERROR: Could not open GPIO 35: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_OpenGPIO35;
    }

    // Open button GPIO as input, and set up a timer to poll it
    Log_Debug("Opening BUTTON_A as input.\n");
    gpioButtonFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_A);
    if (gpioButtonFd < 0) {
        Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_OpenButton;
    }
    struct timespec buttonPressCheckPeriod1Ms = {.tv_sec = 0, .tv_nsec = 1000 * 1000};
    buttonPollTimer = CreateEventLoopPeriodicTimer(eventLoop, ButtonTimerEventHandler,
                                                   &buttonPressCheckPeriod1Ms);
    if (buttonPollTimer == NULL) {
        return ExitCode_Init_ButtonPollTimer;
    }

    return ExitCode_Success;
}

/// <summary>
///     Closes a file descriptor and prints an error on failure.
/// </summary>
/// <param name="fd">File descriptor to close</param>
/// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(buttonPollTimer);
    DisposeEventLoopTimer(MCP39F511PollTimer);
    EventLoop_UnregisterIo(eventLoop, uartEventReg);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors.\n");
    CloseFdAndPrintError(gpioButtonFd, "GpioButton");
    CloseFdAndPrintError(pwrMeterEnFd, "GPIO35");
    CloseFdAndPrintError(uartFd, "Uart");
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("UART application starting.\n");
    exitCode = InitPeripheralsAndHandlers();
  
    // Use event loop to wait for events and trigger handlers, until an error or SIGTERM happens
    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}