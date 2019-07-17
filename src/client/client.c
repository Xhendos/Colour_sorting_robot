#include <string.h>
#include <stdlib.h>

#include "stm32f103xb.h"
#include "client.h"
#include "ax.h"
#include "octo.h"
#include "algo.h"
#include "arm_server.h"
#include "rgb_server.h"

TaskHandle_t xClientTask;

static BaseType_t prvDoColoursMatch(RgbColours_t xColourA, RgbColours_t xColourB);

void vTaskClient( void * pvParameters )
{
RgbColours_t xPlaceholderColoursFirstRound[12];
RgbColours_t xPlaceholderColoursSecondRound[12];
RgbColours_t xPlaceholderColourFirst;
RgbColours_t xPlaceholderColourSecond;
BaseType_t xMatchingPlaceholderColour;
ePlaceholder ePlaceholdersFrom[4];
ePlaceholder ePlaceholdersTo[4];
BaseType_t xPlaceholdersIndex = 0;
static const RgbColours_t xEmptyPlaceholderColour = { 135, 80, 50 };
DisplaceInformation_t xDisplaceInformations[64];
RgbServerMessage_t xRgbServerMessage;
QueueHandle_t xQueueForRgbServerResponse;
UBaseType_t uxAmountOfDisplaceInformation;
UBaseType_t uxAllowedForBuffering;
ArmServerMessage_t xArmServerMessage;
UBaseType_t uxAmountOfExecutedDisplaceInformation;
DisplaceInformation_t * pxDisplaceInformation;
DisplaceInformation_t * pxPreviousDisplaceInformation;
unsigned char ucDisplaceInformationExecuted[64];
unsigned char ucDisplaceInformationBuffered[64];
volatile UBaseType_t uxButtonState;

    xQueueForRgbServerResponse = xQueueCreate( 1, sizeof(RgbColours_t) );

    xRgbServerMessage.xQueueDestination = xQueueForRgbServerResponse;

    while (1)
    {
        while (!(uxArmServerDoneConfiguring && uxRgbServerDoneConfiguring));

        GPIOB->BSRR = (GPIO_BSRR_BS10 | GPIO_BSRR_BR11);

        /* Wait for button press. */
        do { uxButtonState = GPIOB->IDR & GPIO_IDR_IDR8; } while (uxButtonState);
        do { uxButtonState = !(GPIOB->IDR & GPIO_IDR_IDR8); } while (uxButtonState);

        GPIOB->BSRR = (GPIO_BSRR_BS10 | GPIO_BSRR_BS11);

        /* Request the colour of each rgb sensor from the rgb server. The user will have put balls on placeholders before pressing the button. */
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            xRgbServerMessage.ePlaceholder = ePlaceholder;
            xQueueSend( xToRgbServer, &xRgbServerMessage, portMAX_DELAY );
            xQueueReceive( xQueueForRgbServerResponse, &xPlaceholderColoursFirstRound[ePlaceholder], portMAX_DELAY );
        }

        GPIOB->BSRR = (GPIO_BSRR_BS10 | GPIO_BSRR_BR11);

        /* Wait for button press. */
        do { uxButtonState = GPIOB->IDR & GPIO_IDR_IDR8; } while (uxButtonState);
        do { uxButtonState = !(GPIOB->IDR & GPIO_IDR_IDR8); } while (uxButtonState);

        GPIOB->BSRR = (GPIO_BSRR_BS10 | GPIO_BSRR_BS11);

        /* Request the colour of each rgb sensor from the rgb server. The user might have moved some balls before pressing the button. */
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            xRgbServerMessage.ePlaceholder = ePlaceholder;
            xQueueSend( xToRgbServer, &xRgbServerMessage, portMAX_DELAY );
            xQueueReceive( xQueueForRgbServerResponse, &xPlaceholderColoursSecondRound[ePlaceholder], portMAX_DELAY );
        }

        /* Terminate if there are colours from the second request that do not match any of the first request. */
        for (ePlaceholder ePlaceholderSecond = ePlaceholder0; ePlaceholderSecond <= ePlaceholder11; ++ePlaceholderSecond)
        {
            xPlaceholderColourSecond = xPlaceholderColoursSecondRound[ePlaceholderSecond];
            if (prvDoColoursMatch(xPlaceholderColourSecond, xEmptyPlaceholderColour))
            {
                continue;
            }
            xMatchingPlaceholderColour = 0;
            for (ePlaceholder ePlaceholderFirst = ePlaceholder0; ePlaceholderFirst <= ePlaceholder11; ++ePlaceholderFirst)
            {
                xPlaceholderColourFirst = xPlaceholderColoursFirstRound[ePlaceholderFirst];
                if (prvDoColoursMatch(xPlaceholderColourFirst, xEmptyPlaceholderColour))
                {
                    continue;
                }
                if (prvDoColoursMatch(xPlaceholderColourFirst, xPlaceholderColourSecond))
                {
                    ePlaceholdersFrom[xPlaceholdersIndex] = ePlaceholderSecond;
                    ePlaceholdersTo[xPlaceholdersIndex] = ePlaceholderFirst;
                    ++xPlaceholdersIndex;
                    xMatchingPlaceholderColour = 1;
                    break;
                }
            }
            if (!xMatchingPlaceholderColour)
            {
                GPIOB->BSRR = (GPIO_BSRR_BR10 | GPIO_BSRR_BS11);

                /* Terminate. */
                while (1);
            }
        }

        /* Move all arms to their rest position. */
        xArmServerMessage.eArms = ALL_ARMS;
        xArmServerMessage.eMovement = eRest;
        xArmServerMessage.eExecute = eDoExecute;
        xArmServerMessage.xSenderOfMessage = xClientTask;
        xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        /* Use algorithm to find displace information based on the begin and end positions of the balls. Process all of the displace information by requesting the arm server to move certain arms. Each displace information contains information about which arm to displace a ball and the rotation (goal position) in degrees the arm needs to turn to the 'from' placeholder and to the 'to' placeholder. The arms that are allowed to displace a ball are buffered into a round and executed at the same time. */
        uxAmountOfDisplaceInformation = usAlgorithmEntryPoint( ePlaceholdersFrom, ePlaceholdersTo, xDisplaceInformations );
        uxAmountOfExecutedDisplaceInformation = 0;
        memset(&ucDisplaceInformationBuffered, 0, sizeof(ucDisplaceInformationExecuted));
        memset(&ucDisplaceInformationExecuted, 0, sizeof(ucDisplaceInformationExecuted));

        do
        {
            for (UBaseType_t uxCurrentDisplaceInformation = 0; uxCurrentDisplaceInformation < uxAmountOfDisplaceInformation; ++uxCurrentDisplaceInformation)
            {
                pxDisplaceInformation = &xDisplaceInformations[uxCurrentDisplaceInformation];

                if (ucDisplaceInformationExecuted[uxCurrentDisplaceInformation])
                {
                    continue;
                }

                uxAllowedForBuffering = 1;

                for (UBaseType_t uxPreviousDisplaceInformation = 0; uxPreviousDisplaceInformation < uxCurrentDisplaceInformation; ++uxPreviousDisplaceInformation)
                {
                    pxPreviousDisplaceInformation = &xDisplaceInformations[uxPreviousDisplaceInformation];

                    if (ucDisplaceInformationExecuted[uxPreviousDisplaceInformation])
                    {
                        continue;
                    }

                    if (pxDisplaceInformation->ePlaceholderFrom == pxPreviousDisplaceInformation->ePlaceholderFrom
                        || pxDisplaceInformation->ePlaceholderFrom == pxPreviousDisplaceInformation->ePlaceholderTo
                        || pxDisplaceInformation->ePlaceholderTo == pxPreviousDisplaceInformation->ePlaceholderFrom
                        || pxDisplaceInformation->ePlaceholderTo == pxPreviousDisplaceInformation->ePlaceholderTo)
                    {
                        uxAllowedForBuffering = 0;
                        break;
                    }
                }

                if (!uxAllowedForBuffering)
                {
                    continue;
                }

                xArmServerMessage.eArms = 1 << pxDisplaceInformation->ucArm;
                xArmServerMessage.eMovement = eDisplace;
                xArmServerMessage.eExecute = eDontExecute;
                xArmServerMessage.usFirstRotationInDegrees = pxDisplaceInformation->usFirstRotationInDegrees;
                xArmServerMessage.usSecondRotationInDegrees = pxDisplaceInformation->usSecondRotationInDegrees;
                xArmServerMessage.xSenderOfMessage = xClientTask;
                xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );

                ucDisplaceInformationBuffered[uxCurrentDisplaceInformation] = 1;
            }

            xArmServerMessage.eArms = 0;
            xArmServerMessage.eMovement = eDisplace;
            xArmServerMessage.eExecute = eDoExecute;
            xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );
            ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

            for (UBaseType_t uxI = 0; uxI < uxAmountOfDisplaceInformation; ++uxI)
            {
                if (!ucDisplaceInformationExecuted[uxI] && ucDisplaceInformationBuffered[uxI])
                {
                    ucDisplaceInformationExecuted[uxI] = 1;
                    ++uxAmountOfExecutedDisplaceInformation;
                }
            }
        } while (uxAmountOfExecutedDisplaceInformation != uxAmountOfDisplaceInformation);
    }
}

static BaseType_t prvDoColoursMatch(RgbColours_t xColourA, RgbColours_t xColourB)
{
    static UBaseType_t uxMargin = 8;

    if (xColourA.ulRed - xColourB.ulRed > uxMargin)
    {
        return 0;
    }

    if (xColourA.ulGreen - xColourB.ulGreen > uxMargin)
    {
        return 0;
    }

    if (xColourA.ulBlue - xColourB.ulBlue > uxMargin)
    {
        return 0;
    }

    return 1;
}

