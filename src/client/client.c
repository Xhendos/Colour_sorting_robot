#include <string.h>

#include "stm32f103xb.h"
#include "client.h"
#include "ax.h"
#include "octo.h"
#include "algo.h"
#include "arm_server.h"
#include "rgb_server.h"

TaskHandle_t xClientTask;

static BaseType_t prvDoColoursMatch(BaseType_t xColourA, BaseType_t xColourB);

void vTaskClient( void * pvParameters )
{
BaseType_t xPlaceholderColoursFirstRound[12];
BaseType_t xPlaceholderColoursSecondRound[12];
BaseType_t xPlaceholderColourFirst;
BaseType_t xPlaceholderColourSecond;
BaseType_t xMatchingPlaceholderColour;
ePlaceholder ePlaceholdersFrom[4];
ePlaceholder ePlaceholdersTo[4];
BaseType_t xPlaceholdersIndex = 0;
BaseType_t xEmptyPlaceholderColour = 0;
DisplaceInformation_t xDisplaceInformations[64];

    while (1)
    {
        //Wait for button press.
        //while (GPIOB->IDR & GPIO_IDR_IDR8);
        //while (!(GPIOB->IDR & GPIO_IDR_IDR8));
        //Scan each placeholder for colours.
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            //Send message to rgb server queue.
            //Receive response.
            //Process response.
            xPlaceholderColoursFirstRound[ePlaceholder] = 0;
        }
        //Wait for button press.
        //while (GPIOB->IDR & GPIO_IDR_IDR8);
        //while (!(GPIOB->IDR & GPIO_IDR_IDR8));
        //Scan each placeholder for colours.
        for (ePlaceholder ePlaceholder = ePlaceholder0; ePlaceholder <= ePlaceholder11; ++ePlaceholder)
        {
            //Send message to rgb server queue.
            //Receive response.
            //Process response.
            xPlaceholderColoursSecondRound[ePlaceholder] = 0;
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
        ePlaceholdersTo[0] = octoF0;
        ePlaceholdersTo[1] = octoT2;
        ePlaceholdersTo[2] = octoF2;
        ePlaceholdersTo[3] = octoF3;

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
        xQueueSend( xArmServerMessageQueue, &xArmServerMessage, portMAX_DELAY );
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

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

static BaseType_t prvDoColoursMatch(BaseType_t xColourA, BaseType_t xColourB)
{
    if (xColourA == xColourB)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

