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

    xQueueForRgbServerResponse = xQueueCreate( 1, sizeof(RgbColours_t) );

    xRgbServerMessage.xQueueDestination = xQueueForRgbServerResponse;

    while (1)
    {
        //Wait for button press.
        //while (GPIOB->IDR & GPIO_IDR_IDR8);
        //while (!(GPIOB->IDR & GPIO_IDR_IDR8));
        //Scan each placeholder for colours.
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            //Send message to rgb server queue.
            xRgbServerMessage.ePlaceholder = ePlaceholder;
            xQueueSend( xToRgbServer, &xRgbServerMessage, portMAX_DELAY );
            //Receive response.
            xQueueReceive( xQueueForRgbServerResponse, &xPlaceholderColoursFirstRound[ePlaceholder], portMAX_DELAY );
        }
        //Wait for button press.
        //while (GPIOB->IDR & GPIO_IDR_IDR8);
        //while (!(GPIOB->IDR & GPIO_IDR_IDR8));
        //Scan each placeholder for colours.
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            //Send message to rgb server queue.
            xRgbServerMessage.ePlaceholder = ePlaceholder;
            xQueueSend( xToRgbServer, &xRgbServerMessage, portMAX_DELAY );
            //Receive response.
            xQueueReceive( xQueueForRgbServerResponse, &xPlaceholderColoursSecondRound[ePlaceholder], portMAX_DELAY );
        }
        //Terminate if there are balls with a colour that do not match any of the first reading.
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
                //Terminate.
            }
        }

        //Go through all edges and buffer as many movements as possible in a round.
        ePlaceholdersFrom[0] = octoT0;
        ePlaceholdersFrom[1] = octoT2;
        ePlaceholdersFrom[2] = octoT4;
        ePlaceholdersFrom[3] = octoT6;
        ePlaceholdersTo[0] = octoF2;
        ePlaceholdersTo[1] = octoF3;
        ePlaceholdersTo[2] = octoF0;
        ePlaceholdersTo[3] = octoF1;

        UBaseType_t uxAmountOfDisplaceInformation = usAlgorithmEntryPoint( ePlaceholdersFrom, ePlaceholdersTo, xDisplaceInformations );
        UBaseType_t uxAllowedForBuffering;
        ArmServerMessage_t xArmServerMessage;
        UBaseType_t uxAmountOfExecutedDisplaceInformation = 0;
        DisplaceInformation_t * pxDisplaceInformation;
        DisplaceInformation_t * pxPreviousDisplaceInformation;
        unsigned char ucDisplaceInformationExecuted[64];
        unsigned char ucDisplaceInformationBuffered[64];

        memset(&ucDisplaceInformationBuffered, 0, sizeof(ucDisplaceInformationExecuted));
        memset(&ucDisplaceInformationExecuted, 0, sizeof(ucDisplaceInformationExecuted));

        xArmServerMessage.eArms = ALL_ARMS;
        xArmServerMessage.eMovement = eRest;
        xArmServerMessage.eExecute = eDoExecute;
        xArmServerMessage.xSenderOfMessage = xClientTask;
        //xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );
        //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

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
                //xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );

                ucDisplaceInformationBuffered[uxCurrentDisplaceInformation] = 1;
            }

            xArmServerMessage.eArms = 0;
            xArmServerMessage.eMovement = eDisplace;
            xArmServerMessage.eExecute = eDoExecute;
            //xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );
            //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

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
    static UBaseType_t uxMargin = 30;

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

